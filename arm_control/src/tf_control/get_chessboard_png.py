from PIL import Image, ImageDraw

cols = 8
rows = 5
square_size = 40

def create_checkerboard(square_size, cols, rows, square_color1, square_color2, output_path):
    width = cols * square_size
    height = rows * square_size
    checkerboard = Image.new('RGB', (width, height))
    draw = ImageDraw.Draw(checkerboard)

    for i in range(cols):
        for j in range(rows):
            top_left = (i * square_size, j * square_size)
            bottom_right = ((i + 1) * square_size, (j + 1) * square_size)
            color = square_color1 if (i + j) % 2 == 0 else square_color2
            draw.rectangle([top_left, bottom_right], fill=color)

    checkerboard.save(output_path)
    print(f"Checkerboard image saved to {output_path}")

# Example usage
create_checkerboard(
    square_size=square_size * 10,
    cols=cols,
    rows=rows,                 
    square_color1=(255, 255, 255), 
    square_color2=(0, 0, 0), 
    output_path='src/arm_control/chessboard/checkerboard.png'
)

