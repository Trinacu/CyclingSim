
# Example usage of the grid spritesheet:
# Assuming you have these arrays from the manifest:

sprite_x_positions = [0, 50, 100, ...]  # From manifest
sprite_y_positions = [0, 0, 0, ...]     # From manifest
sprite_widths = [48, 48, 48, ...]       # From manifest
sprite_heights = [48, 48, 48, ...]      # From manifest
IMAGES_PER_ROW = 6

# To get sprite at grid position (column, row):
def get_sprite_position(col, row, sprite_index=0):
    """Get position of sprite at grid coordinates or by index"""
    if sprite_index > 0:
        # By index
        return (sprite_x_positions[sprite_index], 
                sprite_y_positions[sprite_index],
                sprite_widths[sprite_index],
                sprite_heights[sprite_index])
    else:
        # By grid position
        index = row * IMAGES_PER_ROW + col
        if index < len(sprite_x_positions):
            return (sprite_x_positions[index], 
                    sprite_y_positions[index],
                    sprite_widths[index],
                    sprite_heights[index])
        return None

# Iterate through all sprites:
for i in range(len(sprite_x_positions)):
    x = sprite_x_positions[i]
    y = sprite_y_positions[i]
    width = sprite_widths[i]
    height = sprite_heights[i]
    # Draw sprite from spritesheet: (x, y, x+width, y+height)
