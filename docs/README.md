# Trapezoidal Motion Planner Documentation

This directory contains comprehensive documentation for the Trapezoidal Motion Planner.

## Documentation Contents

### [Test Results and Performance Analysis](test-results.md)
Detailed analysis of the planner's performance across various test scenarios including:
- Basic trajectory generation with different plant models
- Velocity mode operation and tracking
- Dynamic preemption (shortcutting) capabilities
- Performance benchmarks and computational analysis
- 20+ plots demonstrating real test data
- Quantitative metrics and validation results

## Quick Links

- **[Main README](../README.md)**: Project overview, build instructions, and usage examples
- **[Test Implementation](../test/planner_tests/)**: Source code for all test scenarios
- **[Generated Plots](../test/planner_tests/corpus/)**: All performance plots in PNG format
- **[API Documentation](../include/trapezoidal_motion_planner/)**: Header files with detailed API documentation

## Contributing to Documentation

When adding new documentation:

1. Follow the existing markdown format and structure
2. Include relevant plots and performance data when applicable
3. Add appropriate cross-references to related sections
4. Update this index file with new documentation links
5. Ensure plots are saved to the appropriate corpus directory

## Building Documentation

To regenerate plots and test data:

```bash
# Build and run tests
cd build
cmake --build .
cd test/planner_tests
./planner_tests

# Generate plots
cd ../../test/planner_tests/py
python3 plot_tg.py
```

This will update all plots in the `test/planner_tests/corpus/` directory referenced by the documentation.