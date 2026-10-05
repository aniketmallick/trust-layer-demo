import sys
from pathlib import Path

sys.dont_write_bytecode = True
EXP = Path(__file__).resolve().parent.parent
if str(EXP) not in sys.path:
    sys.path.insert(0, str(EXP))
