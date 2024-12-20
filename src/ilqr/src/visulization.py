import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle

# Function to draw the rectangle and ellipse
def draw_rectangle_and_ellipse(rect_a, rect_b, ellipse_L, ellipse_W):
    # Create figure and axis
    fig, ax = plt.subplots()

    # Draw rectangle (centered at origin)
    rectangle = Rectangle((-rect_a / 2, -rect_b / 2), rect_a, rect_b, edgecolor='r', facecolor='none', lw=2)
    ax.add_patch(rectangle)

    # Draw ellipse (centered at origin)
    ellipse = Ellipse((0, 0), ellipse_L, ellipse_W, edgecolor='b', facecolor='none', lw=2)
    ax.add_patch(ellipse)

    # Set limits and aspect ratio
    ax.set_xlim([-max(ellipse_L, rect_a), max(ellipse_L, rect_a)])
    ax.set_ylim([-max(ellipse_W, rect_b), max(ellipse_W, rect_b)])
    ax.set_aspect('equal')
    ax.set_title(f"Rectangle (red) with dimensions a={rect_a}, b={rect_b}\nEllipse (blue) with axes L={ellipse_L}, W={ellipse_W}")

    # Show the plot
    plt.grid(True)
    plt.show()

# Define the dimensions of the rectangle and the ellipse
rect_a = 4.79  # Length of the rectangle
rect_b = 2.16  # Width of the rectangle
ellipse_L = (4.79/2 + 1.3)*2  # Major axis of the ellipse
ellipse_W = (2.16/2 + 1.1)*2  # Minor axis of the ellipse

# Call the function to draw the rectangle and the ellipse
draw_rectangle_and_ellipse(rect_a, rect_b, ellipse_L, ellipse_W)
