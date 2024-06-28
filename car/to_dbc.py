import os
from process_dbc import process

def to_dbc():
  process("opendbc/can/PDC_USS.dbc", "opendbc/can/PDC_USS.cc")

if __name__ == "__main__":
  to_dbc()