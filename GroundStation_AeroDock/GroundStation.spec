# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_submodules

project_root = r'D:\地面站'
app_root = r'D:\地面站\ground_station'

hiddenimports = []
hiddenimports += collect_submodules('ui')
hiddenimports += collect_submodules('protocol')
hiddenimports += collect_submodules('models')
hiddenimports += [
    'PyQt5.QtOpenGL',
    'OpenGL',
    'OpenGL.GL',
    'OpenGL.GLU',
    'serial',
    'serial.tools.list_ports',
    'hid',
]

a = Analysis(
    [r'D:\地面站\ground_station\main.py'],
    pathex=[project_root, app_root],
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
    name='地面站',
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
    name='地面站',
)
