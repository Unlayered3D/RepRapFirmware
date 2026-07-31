/*
 * Grid.h
 *
 *  Created on: 18 Nov 2016
 *      Author: David
 */

#ifndef SRC_MOVEMENT_GRID_H_
#define SRC_MOVEMENT_GRID_H_

#include "RepRapFirmware.h"
#include "ObjectModel/ObjectModel.h"

class DataTransfer;
class Deviation;

// The scheme used to interpolate between grid points when evaluating the height map.
// 'linear' is bilinear, which is C0 continuous only: the slope of the compensated surface changes abruptly at every grid line.
// 'cubic' is bicubic Catmull-Rom, which is C1 (tangent) continuous everywhere and still passes through every probed point.
enum class MeshInterpolation : uint8_t { linear = 0, cubic = 1 };

// This class defines the bed probing grid
class GridDefinition INHERIT_OBJECT_MODEL
{
public:
	friend class DataTransfer;
	friend class HeightMap;

	GridDefinition() noexcept;

	char GetAxisLetter(size_t axis) const noexcept pre(axis < 2) { return letters[axis]; }
	uint8_t GetAxisNumber(size_t axis) const noexcept pre(axis < 2) { return axisNumbers[axis]; }
	float GetMin(size_t axis) const noexcept pre(axis < 2) { return mins[axis]; }
	float GetMax(size_t axis) const noexcept pre(axis < 2) { return maxs[axis]; }
	float GetSpacing(size_t axis) const noexcept pre(axis < 2) { return spacings[axis]; }

	uint32_t NumAxisPoints(size_t axis) const noexcept pre(axis < 2) { return nums[axis]; }
	uint32_t NumPoints() const noexcept { return nums[0] * nums[1]; }
	float GetCoordinate(size_t axis, size_t coordinateIndex) const noexcept pre(axis < 2) { return mins[axis] + (coordinateIndex * spacings[axis]); }
	bool IsValid() const noexcept { return isValid; }

	bool Set(const char axisLetters[2], const float axis0Range[2], const float axis1Range[2], float pRadius, const float pSpacings[2]) noexcept;
	void PrintParameters(const StringRef& r) const noexcept;
	void WriteHeadingAndParameters(const StringRef& r) const noexcept;
	static int CheckHeading(const StringRef& s) noexcept;
	bool ReadParameters(const StringRef& s, int version) noexcept;

	void PrintError(float originalXrange, float originalYrange, const StringRef& r) const noexcept
	pre(!IsValid());

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	bool IsInRadius(float x, float y) const noexcept;
	void CheckValidity(bool setNum0Num1) noexcept;

	static constexpr float MinSpacing = 0.1;						// The minimum point spacing allowed
	static constexpr float MinRange = 1.0;							// The minimum X and Y range allowed
	static const char *_ecv_array const HeightMapLabelLines[];		// The line we write to the height map file listing the parameter names

	// Primary parameters
	char letters[2];												// Axis letters for this grid
	float mins[2], maxs[2];											// The edges of the grid for G29 probing
	float radius;													// The grid radius to probe
	float spacings[2];												// The spacings of the grid probe points

	// Derived parameters
	uint8_t axisNumbers[2];											// Axis numbers for this grid
	uint32_t nums[2];												// Number of probe points in each direction
	float recipAxisSpacings[2];										// Reciprocals of the axis spacings
	bool isValid;
};

// Class to represent the height map
class HeightMap
{
public:
	HeightMap() noexcept;

	const GridDefinition& GetGrid() const noexcept { return def; }
	void SetGrid(const GridDefinition& gd) noexcept;

	float GetInterpolatedHeightError(float axis0, float axis1) const noexcept;			// Compute the interpolated height error at the specified point
	void ClearGridHeights() noexcept;													// Clear all grid height corrections
	void SetGridHeight(size_t axis0Index, size_t axis1Index, float height) noexcept;	// Set the height of a grid point

	bool IsValid() const noexcept { return def.IsValid(); }

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool SaveToFile(FileStore *f, const char *_ecv_array fname, float zOffset) noexcept			// Save the grid to file returning true if an error occurred
		pre(IsValid());

	bool LoadFromFile(FileStore *f, const char *_ecv_array fname, const StringRef& r
# if SUPPORT_PROBE_POINTS_FILE
						, bool isPointsFile
# endif
						) noexcept;	// Load the grid from file returning true if an error occurred

	const char *_ecv_array GetFileName() const noexcept { return fileName.c_str(); }
#endif

	unsigned int GetMinimumSegments(float deltaAxis0, float deltaAxis1) const noexcept;	// Return the minimum number of segments for a move by this X or Y amount

	bool UseHeightMap(bool b) noexcept;
	bool UsingHeightMap() const noexcept { return useMap; }

	// Interpolation scheme and segmentation tolerance. These are machine settings, not map data, so they survive SetGrid and are not stored in the height map file.
	void SetInterpolation(MeshInterpolation m) noexcept { interpolation = m; }
	MeshInterpolation GetInterpolation() const noexcept { return interpolation; }
	const char *_ecv_array GetInterpolationName() const noexcept;
	void SetChordTolerance(float t) noexcept { chordTolerance = (t > 0.0) ? t : 0.0; }
	float GetChordTolerance() const noexcept { return chordTolerance; }

	unsigned int GetStatistics(Deviation& deviation, float& minError, float& maxError) const noexcept;	// Return number of points probed, mean and RMS deviation, min and max error
	bool CanProbePoint(size_t axis0Index, size_t axis1Index) const noexcept;			// Return true if we can probe this point
	void ExtrapolateMissing() noexcept;													// Extrapolate missing points to ensure consistency

#if SUPPORT_PROBE_POINTS_FILE
	void ClearProbePointsInvalid() noexcept { gridPointInvalid.ClearAll(); }
#endif

private:
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	// Increase the version number in the following string whenever we change the format of the height map file significantly.
	// Adding more fields to the header row can be handled in GridDefinition::ReadParameters() though.
	static constexpr const char *_ecv_array HeightMapComment = "RepRapFirmware height map file v2";	// The start of the comment we write at the start of the height map file
# if SUPPORT_PROBE_POINTS_FILE
	static constexpr const char *_ecv_array PointsFileComment = "RepRapFirmware probe points file v2";	// The start of the comment we write at the start of the points map file
# endif
#endif

	GridDefinition def;
	float gridHeights[MaxGridProbePoints];							// The Z coordinates of the points on the bed that were probed
	LargeBitmap<MaxGridProbePoints> gridHeightSet;					// Bitmap of which heights are set
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	String<MaxFilenameLength> fileName;								// The name of the file that this height map was loaded from or saved to
#endif
#if SUPPORT_PROBE_POINTS_FILE
	LargeBitmap<MaxGridProbePoints> gridPointInvalid;				// Bitmap of which points are not valid
#endif
	float chordTolerance;											// Max allowed deviation in mm between the segment chords and the mesh surface, 0 = use the legacy fixed rule
	float maxSecondDiff[2];											// Largest absolute second difference of the grid heights along each axis, used to bound the surface curvature
	MeshInterpolation interpolation;								// How we interpolate between grid points
	bool curvatureValid;											// True if maxSecondDiff is up to date with the current grid heights
	bool useMap;													// True to do bed compensation

	size_t GetMapIndex(size_t axis0Index, size_t axis1Index) const noexcept { return (axis1Index * def.NumAxisPoints(0)) + axis0Index; }
	void SetGridHeight(size_t index, float height) noexcept;							// Set the height of a grid point

	float InterpolateAxis0Axis1(size_t axis0Index, size_t axis1Index, float axis0Frac, float axis1Frac) const noexcept;
	float InterpolateAxis0Axis1Cubic(int axis0Index, int axis1Index, float axis0Frac, float axis1Frac) const noexcept;
	float InterpolateRowCubic(size_t axis1Index, int axis0Index, float axis0Frac) const noexcept;
	void ComputeCurvatureBounds() noexcept;								// Recompute maxSecondDiff from the current grid heights

#if SUPPORT_PROBE_POINTS_FILE
	bool InterpolateMissingPoint(size_t axis0Index, size_t axis1Index, float& height) const noexcept;
#endif
};

#endif /* SRC_MOVEMENT_GRID_H_ */
