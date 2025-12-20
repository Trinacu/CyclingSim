import os
import glob
from PIL import Image

def create_spritesheet_grid():
    # Get all PNG files in current directory
    png_files = glob.glob("*.png")
    
    if not png_files:
        print("No PNG files found in current directory!")
        return
    
    # Sort files alphabetically
    png_files.sort()
    
    print(f"Found {len(png_files)} PNG files:")
    for file in png_files:
        print(f"  - {file}")
    
    # Constants
    IMAGES_PER_ROW = 6
    SCALE_FACTOR = 0.5  # Scale down to 50%
    
    # Open all images, scale them, and track dimensions
    images = []
    scaled_widths = []
    scaled_heights = []
    
    for file in png_files:
        try:
            img = Image.open(file)
            
            # Calculate scaled dimensions
            new_width = int(img.width * SCALE_FACTOR)
            new_height = int(img.height * SCALE_FACTOR)
            
            # Resize the image
            scaled_img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
            images.append(scaled_img)
            scaled_widths.append(new_width)
            scaled_heights.append(new_height)
            
            print(f"Loaded {file}: {img.width}x{img.height} -> {new_width}x{new_height}")
            
            img.close()  # Close original image
            
        except Exception as e:
            print(f"Error loading {file}: {e}")
    
    if not images:
        print("No valid images could be loaded!")
        return
    
    # Calculate spritesheet dimensions
    num_images = len(images)
    num_rows = (num_images + IMAGES_PER_ROW - 1) // IMAGES_PER_ROW  # Ceiling division
    
    # Find maximum width and height in each row
    row_heights = []
    col_widths_per_row = []
    
    for row in range(num_rows):
        start_idx = row * IMAGES_PER_ROW
        end_idx = min(start_idx + IMAGES_PER_ROW, num_images)
        
        # Max height in this row
        row_max_height = max(scaled_heights[start_idx:end_idx])
        row_heights.append(row_max_height)
        
        # Widths of each column in this row
        row_widths = []
        for i in range(start_idx, end_idx):
            row_widths.append(scaled_widths[i])
        col_widths_per_row.append(row_widths)
    
    # Calculate total spritesheet dimensions
    # We need to find max width for each column across all rows
    max_col_widths = [0] * IMAGES_PER_ROW
    
    for row in range(num_rows):
        for col in range(len(col_widths_per_row[row])):
            max_col_widths[col] = max(max_col_widths[col], col_widths_per_row[row][col])
    
    total_width = sum(max_col_widths)
    total_height = sum(row_heights)
    
    print(f"\nCreating {IMAGES_PER_ROW} columns x {num_rows} rows spritesheet")
    print(f"Images scaled to {SCALE_FACTOR * 100}% of original size")
    print(f"Total spritesheet size: {total_width}x{total_height}")
    print(f"Max column widths: {max_col_widths}")
    print(f"Row heights: {row_heights}")
    
    # Create spritesheet
    spritesheet = Image.new('RGBA', (total_width, total_height))
    
    # Keep track of sprite positions for manifest
    sprite_positions = []
    
    # Current y position
    current_y = 0
    
    for row in range(num_rows):
        current_x = 0
        row_height = row_heights[row]
        
        for col in range(IMAGES_PER_ROW):
            sprite_idx = row * IMAGES_PER_ROW + col
            
            if sprite_idx >= num_images:
                break  # No more images
                
            img = images[sprite_idx]
            col_width = max_col_widths[col]
            
            # Create cell with transparency
            cell = Image.new('RGBA', (col_width, row_height), (0, 0, 0, 0))
            
            # Center the image horizontally in its column
            x_offset = (col_width - img.width) // 2
            y_offset = (row_height - img.height) // 2
            
            cell.paste(img, (x_offset, y_offset))
            spritesheet.paste(cell, (current_x, current_y))
            
            # Store position info
            sprite_positions.append({
                'filename': png_files[sprite_idx],
                'x': current_x + x_offset,
                'y': current_y + y_offset,
                'width': img.width,
                'height': img.height,
                'col': col,
                'row': row
            })
            
            print(f"  Placed {png_files[sprite_idx]} at ({col},{row}) position: ({current_x + x_offset}, {current_y + y_offset})")
            
            current_x += col_width
        
        current_y += row_height
    
    # Save spritesheet
    output_filename = "spritesheet_grid_scaled.png"
    spritesheet.save(output_filename, "PNG")
    print(f"\nSpritesheet saved as: {output_filename}")
    print(f"Spritesheet dimensions: {spritesheet.width}x{spritesheet.height}")
    
    # Create manifest file
    create_grid_manifest(sprite_positions, IMAGES_PER_ROW, SCALE_FACTOR, output_filename)
    
    # Clean up
    for img in images:
        img.close()
    
    return spritesheet, sprite_positions

def create_grid_manifest(sprite_data, cols, scale_factor, spritesheet_name):
    """Create a manifest file with sprite positions"""
    manifest = []
    manifest.append(f"Spritesheet: {spritesheet_name}")
    manifest.append(f"Grid layout: {cols} columns per row")
    manifest.append(f"Scale factor: {scale_factor}")
    manifest.append(f"Total sprites: {len(sprite_data)}")
    manifest.append("\nSprite positions (filename, column, row, x, y, width, height):")
    
    # Sort by row then column for readability
    sorted_sprites = sorted(sprite_data, key=lambda s: (s['row'], s['col']))
    
    for sprite in sorted_sprites:
        manifest.append(f"{sprite['filename']}: col={sprite['col']}, row={sprite['row']}, "
                       f"x={sprite['x']}, y={sprite['y']}, "
                       f"width={sprite['width']}, height={sprite['height']}")
    
    # Also create arrays for easy iteration
    manifest.append("\n\nArrays for easy iteration:")
    manifest.append("# By row and column")
    manifest.append("sprite_grid = [")
    
    # Group by row
    max_row = max(s['row'] for s in sprite_data)
    for row in range(max_row + 1):
        row_sprites = [s for s in sorted_sprites if s['row'] == row]
        row_str = f"  # Row {row}"
        manifest.append(row_str)
        
        row_positions = []
        for sprite in row_sprites:
            row_positions.append(f"({sprite['x']}, {sprite['y']})")
        
        if row_positions:
            manifest.append(f"  [{', '.join(row_positions)}],")
    
    manifest.append("]")
    
    # Simple flat arrays
    manifest.append("\n# Flat arrays (all sprites in order)")
    manifest.append("sprite_x_positions = [")
    for sprite in sorted_sprites:
        manifest.append(f"    {sprite['x']},  # {sprite['filename']}")
    manifest.append("]")
    
    manifest.append("\nsprite_y_positions = [")
    for sprite in sorted_sprites:
        manifest.append(f"    {sprite['y']},  # {sprite['filename']}")
    manifest.append("]")
    
    manifest.append("\nsprite_widths = [")
    for sprite in sorted_sprites:
        manifest.append(f"    {sprite['width']},  # {sprite['filename']}")
    manifest.append("]")
    
    manifest.append("\nsprite_heights = [")
    for sprite in sorted_sprites:
        manifest.append(f"    {sprite['height']},  # {sprite['filename']}")
    manifest.append("]")
    
    with open("spritesheet_grid_manifest.txt", "w") as f:
        f.write("\n".join(manifest))
    
    print(f"Manifest saved as: spritesheet_grid_manifest.txt")

def example_usage():
    """Example of how to use the sprite positions in code"""
    example_code = """
# Example usage of the grid spritesheet:
# Assuming you have these arrays from the manifest:

sprite_x_positions = [0, 50, 100, ...]  # From manifest
sprite_y_positions = [0, 0, 0, ...]     # From manifest
sprite_widths = [48, 48, 48, ...]       # From manifest
sprite_heights = [48, 48, 48, ...]      # From manifest
IMAGES_PER_ROW = 6

# To get sprite at grid position (column, row):
def get_sprite_position(col, row, sprite_index=0):
    \"\"\"Get position of sprite at grid coordinates or by index\"\"\"
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
"""
    
    with open("example_usage.py", "w") as f:
        f.write(example_code)
    
    print("Example usage code saved as: example_usage.py")

if __name__ == "__main__":
    spritesheet, sprite_data = create_spritesheet_grid()
    
    # Create example usage file
    example_usage()
    
    print("\nDone! Press Enter to exit...")
    input()
