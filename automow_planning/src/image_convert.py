import sys
if len(sys.argv) < 2:
	sys.exit(0)
from PIL import Image
im = Image.open(sys.argv[1])
im.save(sys.argv[1]+".jpg", "JPEG")