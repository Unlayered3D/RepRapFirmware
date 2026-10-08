#!/usr/bin/env python3
"""Decode an Unlayered piezo probe log (ring.bin / snap-NNNN.bin, format PZL1).

Format and support-bundle contract: Developer-documentation/Piezo-support-log.md

    decode_piezolog.py snap-0003.bin                  # summary
    decode_piezolog.py snap-0003.bin --csv out.csv    # one row per 1 ms sample
    decode_piezolog.py snap-0003.bin --plot out.png   # needs matplotlib
    decode_piezolog.py bundle.zip                     # summarise every .bin in a support bundle

If the sidecar snap-NNNN.txt sits next to the .bin (or in the zip), the type 12 detector is re-run offline from
the raw samples with the logged M558/G31 parameters. The firmware detector is integer-only, so the replay is
bit-exact once its filters have settled (~4 x W ms from the start of the file); the `trig` column logged by the
board and the replayed `rtrig` should then agree sample for sample. A mismatch means the replay is wrong or the
parameters changed mid-window.
"""

import argparse, io, os, re, struct, sys, zipfile

MAGIC = 0x314C5A50
BLOCK = 512
NSAMP = 248
HDR = struct.Struct('<IIIHH')


def read_blocks(data):
    blocks = []
    for off in range(0, len(data) - BLOCK + 1, BLOCK):
        magic, seq, ms, count, dropped = HDR.unpack_from(data, off)
        if magic != MAGIC or count > NSAMP:
            continue
        samples = struct.unpack_from('<%dH' % count, data, off + 16)
        blocks.append((seq, ms, dropped, samples))
    blocks.sort(key=lambda b: b[0])           # the ring is written in place, so file order is not time order
    return blocks


def samples_from_blocks(blocks):
    """Yield (t_ms, word, gap_before) in time order. Time comes from each block's millis, 1 ms per sample."""
    rows = []
    for seq, ms, dropped, samples in blocks:
        for i, w in enumerate(samples):
            rows.append((ms + i, w, dropped if i == 0 else 0))
    return rows


def parse_sidecar(text):
    p = {}
    m = re.search(r'threshold \(G31 P\): (-?\d+)', text)
    if m: p['P'] = int(m.group(1))
    m = re.search(r'probe type: (\d+)', text)
    if m: p['type'] = int(m.group(1))
    m = re.search(r'filter (raw|delta), V(\d+)ms W(\d+)ms E(\d+)ms O(\d+) \(\d+dB/oct\) J(\d+) \(\d+dB/oct\), D(\d+), Q(\d+)ms, U(\d+)ms', text)
    if m:
        p.update(L=1 if m.group(1) == 'raw' else 0, V=int(m.group(2)), W=int(m.group(3)), E=int(m.group(4)),
                 O=int(m.group(5)), J=int(m.group(6)), D=int(m.group(7)), Q=int(m.group(8)), U=int(m.group(9)))
    p['reason'] = (re.search(r'reason: (.*)', text) or [None, '?'])[1]
    return p


def millis_to_shift(ms):
    ms = max(1, min(ms, 4096))
    sh = ms.bit_length() - 1
    if sh < 12 and (ms - (1 << sh)) > ((1 << (sh + 1)) - ms):
        sh += 1
    return min(sh, 12)


class Detector:
    """Python port of ZProbe::ProcessDifferentialSample / ArmDifferential (ZProbe.cpp). Keep in step with it."""
    FRAC = 12

    def __init__(self, p):
        self.p = p
        self.order = max(1, min(p['O'], 4)); self.hporder = max(1, min(p['J'], 4))
        self.fs = millis_to_shift(p['V']); self.ss = millis_to_shift(p['W'])
        self.ish = millis_to_shift(max(1, p['E'] // self.order))
        self.primed = False
        self.trig = 0; self.hold = 0; self.blank = 0; self.prime = 0
        self.probing = False; self.base = 0
        self.fast = self.slow = 0; self.lp = [0] * 4; self.hp = [0] * 4

    def arm(self, probing):
        self.probing = probing
        self.base = self.fast >> self.FRAC
        self.blank = self.p['U'] if probing else 0
        self.hold = 0; self.trig = 0

    def step(self, raw):
        F = self.FRAC; x = raw << F; p = self.p
        if not self.primed:
            self.fast = self.slow = x
            self.lp = [x if p['L'] == 1 else 0] * 4
            self.hp = [x, 0, 0, 0]
            self.prime = min(4 << self.ss, 8000); self.hold = self.blank = 0; self.trig = 0
            self.primed = True
            return 0, 0, 0
        self.fast += (x - self.fast) >> self.fs
        self.slow += (x - self.slow) >> self.ss
        delta = self.fast - self.slow
        v = self.fast
        if p['L'] == 1:
            for k in range(self.order): self.lp[k] += (v - self.lp[k]) >> self.ish; v = self.lp[k]
            for k in range(self.hporder): self.hp[k] += (v - self.hp[k]) >> self.ss; v -= self.hp[k]
        else:
            for k in range(self.hporder): self.hp[k] += (v - self.hp[k]) >> self.ss; v -= self.hp[k]
            for k in range(self.order): self.lp[k] += (v - self.lp[k]) >> self.ish; v = self.lp[k]
        d = max(-32000, min(32000, delta >> F)); f = max(-32000, min(32000, v >> F))
        if self.prime: self.prime -= 1; return d, f, self.trig
        if self.blank: self.blank -= 1; return d, f, self.trig
        sign = 0; P = p['P']; D = p['D']
        if D != 2 and f >= P: sign = 1
        elif D != 1 and f <= -P: sign = -1
        if sign == 0 and self.probing:
            dev = (self.fast >> F) - self.base
            if D != 2 and dev >= P: sign = 1
            elif D != 1 and dev <= -P: sign = -1
        if sign:
            self.hold = p['Q']; self.trig = sign
        elif self.hold:
            self.hold -= 1
            if self.hold == 0: self.trig = 0
        return d, f, self.trig


def decode(name, data, sidecar_text=None):
    blocks = read_blocks(data)
    rows = samples_from_blocks(blocks)
    params = parse_sidecar(sidecar_text) if sidecar_text else {}
    out = []
    det = Detector(params) if 'V' in params else None
    settle = (4 << millis_to_shift(params['W'])) + 50 if det else 0
    was_probing = False
    for idx, (t, w, gap) in enumerate(rows):
        raw = w & 0x3FF
        trig = (-1 if w & 0x800 else 1) if w & 0x400 else 0
        probing = bool(w & 0x1000)
        row = dict(t=t, raw=raw, trig=trig, probing=int(probing), blank=int(bool(w & 0x2000)),
                   priming=int(bool(w & 0x4000)), gap=gap)
        if det:
            if gap: det.primed = False                      # samples were lost, so the replay restarts
            if probing != was_probing: det.arm(probing)     # the board arms in SetProbing, just before the first flagged tick
            d, f, rt = det.step(raw)
            row.update(delta=d, filtered=f, rtrig=rt, settled=int(idx >= settle))
        was_probing = probing
        out.append(row)
    return blocks, params, out


def summarise(name, blocks, params, rows):
    print(f'== {name}')
    if not rows:
        print('   no valid blocks'); return
    t0, t1 = rows[0]['t'], rows[-1]['t']
    raws = [r['raw'] for r in rows]
    gaps = sum(r['gap'] for r in rows)
    print(f'   {len(blocks)} blocks, {len(rows)} samples, {(t1 - t0) / 1000:.1f} s (millis {t0}..{t1}), dropped {gaps}')
    if params: print(f"   reason: {params.get('reason')}, probe type {params.get('type')}, threshold {params.get('P')}")
    print(f'   raw min {min(raws)} max {max(raws)} mean {sum(raws) / len(raws):.1f}')
    edges = [r for i, r in enumerate(rows) if r['trig'] and (i == 0 or not rows[i - 1]['trig'])]
    arms = [r for i, r in enumerate(rows) if r['probing'] and (i == 0 or not rows[i - 1]['probing'])]
    print(f'   probing moves {len(arms)}, trigger edges {len(edges)}')
    for r in edges[:20]:
        print(f"     trig {'+' if r['trig'] > 0 else '-'} at {r['t']}  raw {r['raw']}  probing {r['probing']}")
    if 'rtrig' in rows[0]:
        cmp = [r for r in rows if r['settled']]
        bad = sum(1 for r in cmp if (r['rtrig'] != 0) != (r['trig'] != 0))
        print(f'   offline replay vs board trigger: {bad} mismatching samples of {len(cmp)} settled')


def plot(rows, path, title, threshold=None):
    try:
        import matplotlib
    except ImportError:
        print('   --plot needs matplotlib: pip install matplotlib'); return
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = [(r['t'] - rows[0]['t']) / 1000 for r in rows]
    fig, ax = plt.subplots(2 if 'filtered' in rows[0] else 1, 1, figsize=(14, 7), sharex=True, squeeze=False)
    a = ax[0][0]
    a.plot(t, [r['raw'] for r in rows], lw=0.6, label='raw')
    a.fill_between(t, 0, 1023, where=[bool(r['probing']) for r in rows], alpha=0.12, label='probing', step='mid')
    a.fill_between(t, 0, 1023, where=[bool(r['trig']) for r in rows], alpha=0.3, color='red', label='triggered', step='mid')
    lo, hi = min(r['raw'] for r in rows), max(r['raw'] for r in rows)
    a.set_ylim(lo - 20, hi + 20); a.legend(loc='upper right'); a.set_title(title)
    if 'filtered' in rows[0]:
        b = ax[1][0]
        b.plot(t, [r['filtered'] for r in rows], lw=0.6, label='filtered (replayed)')
        b.plot(t, [r['delta'] for r in rows], lw=0.4, alpha=0.6, label='delta (replayed)')
        if threshold is not None:
            b.axhline(threshold, color='k', lw=0.3); b.axhline(-threshold, color='k', lw=0.3)
        b.legend(loc='upper right')
    ax[-1][0].set_xlabel('s')
    fig.tight_layout(); fig.savefig(path, dpi=120)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path')
    ap.add_argument('--csv'); ap.add_argument('--plot')
    a = ap.parse_args()

    items = []
    if zipfile.is_zipfile(a.path):
        z = zipfile.ZipFile(a.path)
        names = z.namelist()
        for n in sorted(names):
            if n.endswith('.bin'):
                side = n[:-4] + '.txt'
                items.append((n, z.read(n), z.read(side).decode('utf-8', 'replace') if side in names else None))
    else:
        side = os.path.splitext(a.path)[0] + '.txt'
        items.append((os.path.basename(a.path), open(a.path, 'rb').read(),
                      open(side, encoding='utf-8', errors='replace').read() if os.path.exists(side) else None))

    for name, data, side in items:
        blocks, params, rows = decode(name, data, side)
        summarise(name, blocks, params, rows)
        if len(items) == 1:
            if a.csv and rows:
                with open(a.csv, 'w', newline='') as f:
                    keys = list(rows[0].keys())
                    f.write(','.join(keys) + '\n')
                    for r in rows: f.write(','.join(str(r[k]) for k in keys) + '\n')
            if a.plot and rows:
                plot(rows, a.plot, f"{name} ({params.get('reason', '?')})", params.get('P'))


if __name__ == '__main__':
    main()
