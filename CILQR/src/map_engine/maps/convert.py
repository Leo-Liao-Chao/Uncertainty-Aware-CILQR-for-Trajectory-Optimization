from PIL import Image

img_file = 'Town02'
img = Image.open(img_file + '.tga')
Gray = img.convert('L')

threshold = 70

table = []
for i in range(256):
    if i > threshold:
        table.append(0)
    else:
        table.append(255)


photo = Gray.point(table, '1')
photo.save(img_file + '.png')