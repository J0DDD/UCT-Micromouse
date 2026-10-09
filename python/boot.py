# boot.py - UCT Micromouse Bootloader
import os, pyb

# Auto-purge macOS metadata bloat (._* and .DS_Store) to preserve flash storage
try:
    for f in os.listdir('/flash'):
        if f.startswith('._') or f in ('.DS_Store', '.Trashes'):
            try:
                os.remove('/flash/' + f)
            except Exception:
                pass
except Exception:
    pass

pyb.main('main.py')


