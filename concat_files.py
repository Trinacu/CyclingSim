import os
import glob

def read_file_contents(filepath):
    """Read and return the contents of a file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as file:
            return file.read()
    except UnicodeDecodeError:
        # Fallback for binary files or different encodings
        try:
            with open(filepath, 'r', encoding='latin-1') as file:
                return file.read()
        except:
            return "[Binary file or unable to read]"

def find_and_process_files():
    """Find all files in src and include directories and write to output file"""
    
    # Define directories to search
    directories = ['src', 'include']
    
    # Find all files in the directories
    all_files = []
    for directory in directories:
        if os.path.exists(directory):
            # Recursively find all files in the directory
            for root, dirs, files in os.walk(directory):
                if 'Eigen' in dirs:
                    dirs.remove('Eigen')
                for file in files:
                    full_path = os.path.join(root, file)
                    all_files.append(full_path)
        else:
            print(f"Warning: Directory '{directory}' not found")
    
    # Sort files for consistent output
    all_files.sort()
    
    # Write to output file
    output_filename = "files_contents.txt"
    
    with open(output_filename, 'w', encoding='utf-8') as output_file:
        for filepath in all_files:
            # Write filename
            output_file.write(f"{filepath}\n")
            
            # Write file contents in code block
            output_file.write("```\n")
            contents = read_file_contents(filepath)
            output_file.write(contents)
            
            # Add newline if file doesn't end with one
            if contents and not contents.endswith('\n'):
                output_file.write('\n')
            
            output_file.write("```\n\n")
    
    print(f"Successfully processed {len(all_files)} files")
    print(f"Output written to: {output_filename}")

if __name__ == "__main__":
    find_and_process_files()
