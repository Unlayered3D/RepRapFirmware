/*
 * MoveSegment.cpp
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 */

#include "MoveSegment.h"

// Static members

MoveSegment *_ecv_null MoveSegment::freeList = nullptr;
unsigned int MoveSegment::numCreated = 0;
unsigned int MoveSegment::numFree = 0;

// Return true if preparing another move cannot run the heap out of memory
bool MoveSegment::HaveRoomToPrepareMove() noexcept
{
	const unsigned int freeSegments = numFree;
	if (freeSegments >= SegmentsNeededToPrepareMove)
	{
		return true;
	}
	const ptrdiff_t spareHeap = Tasks::GetNeverUsedRam() - (ptrdiff_t)HeapReservedForOthers;
	return spareHeap > 0 && freeSegments + (unsigned int)spareHeap/sizeof(MoveSegment) >= SegmentsNeededToPrepareMove;
}

// Allocate a MoveSegment, from the freelist if possible, else create a new one
MoveSegment *MoveSegment::Allocate(MoveSegment *_ecv_null p_next) noexcept
{
	const auto iflags = IrqSave();
	MoveSegment *_ecv_null ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->next;
		--numFree;
		IrqRestore(iflags);
		ms->next = p_next;
	}
	else
	{
		++numCreated;
		IrqRestore(iflags);
		ms = new MoveSegment(p_next);
	}
	return ms;
}

// Release a MoveSegment
void MoveSegment::ReleaseAll(MoveSegment *_ecv_null item) noexcept
{
	while (item != nullptr)
	{
		MoveSegment *itemToRelease = item;
		item = item->next;
		Release(itemToRelease);
	}
}

void MoveSegment::DebugPrint() const noexcept
{
	debugPrintf("s=%" PRIu32 " t=%" PRIu32 " d=%.4f u=%.4e a=%.4e"
#if SUPPORT_3RD_ORDER
				" j=%.4e"
#endif
				" f=%02" PRIx32 "\n",
				startTime, duration, (double)distance, (double)CalcU(), (double)a,
#if SUPPORT_3RD_ORDER
				(double)j,
#endif
				flags.all);
}

// Append details of this segment to a string buffer
void MoveSegment::AppendDetails(const StringRef& str) const noexcept
{
	str.catf("s=%" PRIu32 " t=%" PRIu32 " d=%.4f u=%.4e a=%.4e"
#if SUPPORT_3RD_ORDER
				" j=%.4e"
#endif
				" f=%02" PRIx32 "\n",
				startTime, duration, (double)distance, (double)CalcU(), (double)a,
#if SUPPORT_3RD_ORDER
				(double)j,
#endif
				flags.all);
}

/*static*/ void MoveSegment::DebugPrintList(const MoveSegment *_ecv_null segs) noexcept
{
	if (segs == nullptr)
	{
		debugPrintf("null seg\n");
	}
	else
	{
		while (segs != nullptr)
		{
			segs->DebugPrint();
			segs = segs->GetNext();
		}
	}
}

// End
