import PIL, sys
from struct import *
from PIL import Image

def convert(input, output):
	print input + ' -> ' + output + '...'
	img = PIL.Image.open(input)
	rgb_im = img.convert('RGB')

	w = rgb_im.size[0]
	h = rgb_im.size[1]

	out = open(output, 'w')
	out.write(pack('<H', w)) # Write in Little Endian
	out.write(pack('<H', h)) # Write in Little Endian

	out.write(pack('<H', 0)) # Dummy padding
	out.write(pack('<H', 0)) # Dummy padding

	for y in range(0, rgb_im.size[1]):
		for x in range(0, rgb_im.size[0]):
			p = rgb_im.getpixel((x,y))
			r = p[0] >> 3
			g = p[1] >> 2
			b = p[2] >> 3
			c16 = (r << 11) | (g << 5) | b
			out.write(pack('>H', c16)) # Write in Big Endian

if __name__ == "__main__":
	if len(sys.argv) < 3:
		print 'A small tool that converts an image file to a custom raw binary format for feeding in to the function ST7735R_Draw565() from ST7735R_TFT.h.'
		print 'Usage: ' + sys.argv[0] + ' inputfile.bmp/jpg/png/... outfile.565'
# Hackhackhack..
	for i in range(0, 19440):
		convert('outrun' + ("%05d" % (i,)) + '.bmp', 'out' + ("%05d" % (i,)) + '.565')
#	convert(sys.argv[1], sys.argv[2])
