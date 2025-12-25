import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/gz_simulation/install/simulation'
