/*
 * PiezoLog.h
 *
 *  Created on: 3 Oct 2026
 *      Author: Unlayered3D
 *
 *  A continuous 30 s rolling log of the Z probe (piezo) signal, kept on the SD card so it can be sent to
 *  Unlayered for support. Format and the support-bundle contract: Developer-documentation/Piezo-support-log.md
 *
 *  The ring cannot live in RAM: 30 s at 1 kHz x 2 bytes is 60 KB and a Mini 5+ has ~8 KB that is never used.
 *  So the tick ISR fills three 512-byte blocks and Spin() writes each full block in place into 0:/logs/piezo/ring.bin.
 *  A snapshot freezes the ring by renaming it, so it costs no copy and no RAM.
 */

#ifndef SRC_PLATFORM_PIEZOLOG_H_
#define SRC_PLATFORM_PIEZOLOG_H_

#include <RepRapFirmware.h>

#define SUPPORT_PIEZO_LOG	(HAS_MASS_STORAGE)

#if SUPPORT_PIEZO_LOG

class ZProbe;

namespace PiezoLog
{
	constexpr size_t SamplesPerBlock = 248;			// 16-byte header + 248 x 2 = 512 bytes = one SD sector
	constexpr size_t NumBuffers = 3;				// tolerates an SD write stall of ~0.5 s before samples are dropped
	constexpr size_t RingBlocks = 122;				// 122 x 248 ms = 30.3 s
	constexpr uint32_t BlockMagic = 0x314C5A50;		// "PZL1" little-endian
	constexpr unsigned int MaxSnapshots = 10;

	// Sample word layout
	constexpr uint16_t RawMask = 0x03FF;
	constexpr uint16_t FlagTriggered = 1u << 10;
	constexpr uint16_t FlagTriggerNegative = 1u << 11;
	constexpr uint16_t FlagProbing = 1u << 12;
	constexpr uint16_t FlagBlanking = 1u << 13;
	constexpr uint16_t FlagPriming = 1u << 14;

	struct Block
	{
		uint32_t magic;
		uint32_t seq;
		uint32_t millis;
		uint16_t count;
		uint16_t dropped;
		uint16_t samples[SamplesPerBlock];
	};
	static_assert(sizeof(Block) == 512);

	void TickSample(const ZProbe& zp) noexcept;						// called from the tick ISR, once per tick
	void Spin() noexcept;											// called from Platform::Spin: SD writes, events, snapshots
	void NoteProbing(bool isProbing) noexcept;						// called from LocalZProbe::SetProbing
	void NoteError(const char *_ecv_array msg) noexcept;			// called from GCodeMachineState::SetError; takes a snapshot after a probe failure
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// M1770
	void Diagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_PLATFORM_PIEZOLOG_H_ */
