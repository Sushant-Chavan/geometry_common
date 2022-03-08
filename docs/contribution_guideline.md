# Contribution guidelines

## General coding and formatting guidelines

**Note**: This section is summary of Issue #3. Please check that for further
details.

- Minimum C++ compiler version to be used: C++11

- Header/cpp file naming conventions (camel-case or snake-case)
  - camel-case (Ex. PointcloudProjector.h) if the file defines some class
  - snake-case (Ex. pointcloud_projector.h) if the file does not define any class

- Space margin for access-specifiers in a class
  - 4-point spacing for access specifier
  - 8-point spacing for members (variable and functions)

```c++
class A
{
    public:
        A();

        virtual ~A();

        someFunc();

    protected:
        someOtherFunc();

    private:
        int var_;
        float var_2_;
}
```

- Use 4 spaces for indentation

- Use Javadoc style for Doxygen

- Header guard format 
  - Use Geosoft convention rule 40
  - e.g. `KELO_GEOMETRY_COMMON_LINE_SEGMENT_H`)

- Indentation of class contents
  - Use `BS_Allman` scheme as defined in
    [clang-format](https://clang.llvm.org/docs/ClangFormatStyleOptions.html)

- Class names: camel case
  - e.g.: `PointcloudProjector`, `Point2D`

- Function names (except constructor and destructors): camel case with first
  character not capitalised
  - e.g. `distTo`, `calcIntersectionPointWith`

- Variable names: snake case
  - protected and private member variables should have a trailing `_`
    - e.g.: `mat_`, `passthrough_min_z_`

---

## Data structures

### Function names
- Use `--To` instead of `--From` because it is shorter
  - e.g.: `distTo` instead of `distFrom`
- Functions with return type boolean should be a yes or not question
  - e.g.: `containsPoint`, `isCollinear`
- Don't use `get--` unless it is actually a getter function of a protected or
  private member variable because it is less confusion. If returning a variable
  that requires trivial calculation, just use the name of the quantity.
  - e.g.: in `LineSegment2D` class, `angle()` instead of `getAngle()`
- Don't use `set--` unless it is actually a setter of a protected or private
  member variable because it is less confusing. When setting a variable that
  requires trivial calculation, use `update--` instead.
  - e.g.: in `TransformMatrix2D` class, `updateX()` instead of `setX()`
- when calculating non-trivial quantities, there are two choices for prefix
  - `calc--`
  - `--`
  Here, see which makes more sense in that context and from user's perspective.
  If both make equal sense, use second option since it is shorter.
  - e.g.: In `LineSegment2D` class, `calcIntersectionPointWith`
  - e.g.: In `LineSegment2D` class, `minDistTo`. Here, both `calcMinDistTo` and
    `minDistTo` would have been okay so the shorter one is chosen.
  - If the result is filled in a parameter passed as a reference object then
    always use first option `calc--`.
    - e.g.: `calcIntersectionPointWith`
- when converting from one data type to another in a class member function, use
  `as--` as prefix.
  - e.g.: `asMarker`, `asPoint32`

## Utils

### Function names

- Don't use `get--` as prefix.
- Start with a command word unless the return type is boolean
  - e.g.: `calcMeanPoint`, `applyPiecewiseRegression`
- When return type is boolean, should be a yes or not question.
  - e.g. `isAngleWithinBounds`
