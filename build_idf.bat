@echo off
set MSYSTEM=
set IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.4.1
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.4_py3.11_env
set PATH=C:\Espressif\python_env\idf5.4_py3.11_env\Scripts;C:\Espressif\frameworks\esp-idf-v5.4.1\tools;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;%PATH%
cd /d d:\Fork\WiFi_USB_SD
python C:\Espressif\frameworks\esp-idf-v5.4.1\tools\idf.py build 2>&1
