import os
from process_dbc import process

def to_dbc():
  process("opendbc/can/PDC_USS.dbc", "opendbc/can/1_PDC_12")

if __name__ == "__main__":
  to_dbc()