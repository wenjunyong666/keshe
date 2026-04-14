# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules

hiddenimports = [
    "PyQt5.QtOpenGL",
    "OpenGL",
    "OpenGL.GL",
    "OpenGL.GLU",
    "serial",
    "serial.tools.list_ports",
    "hid",
]
hiddenimports += collect_submodules("ui")
hiddenimports += collect_submodules("protocol")
hiddenimports += collect_submodules("models")


a = Analysis(
    ["D:\\地面站\\ground_station\\main.py"],
    pathex=["D:\\地面站", "D:\\地面站\\ground_station"],
    binaries=[],
    datas=[],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="GroundStationFixed",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="GroundStationFixed",
)
