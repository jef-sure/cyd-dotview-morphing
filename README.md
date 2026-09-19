# CYD + DGX boilerplate

This project is a minimal ESP-IDF starter for a Cheap Yellow Display (CYD) using the DGX graphics component.

## Quick start

```sh
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Included baseline

- ESP-IDF project skeleton
- DGX dependency declaration via `main/idf_component.yml`
- CYD/DGX configuration defaults in `sdkconfig.defaults`
- C++/C formatting profile via `.clang-format`
- VS Code and devcontainer settings for ESP-IDF development

## Main project files

- `main/CMakeLists.txt` — registers the app component and DGX dependencies
- `main/main.c` — application entry point
- `sdkconfig.defaults` — default DSP/target configuration for DGX + ILI9341

## Notes

Adapt the pins and display initialization for your exact CYD revision before moving on to rendering logic.
