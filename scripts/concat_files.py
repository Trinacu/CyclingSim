from pathlib import Path


def read_file_contents(filepath: Path) -> str:
    """Read and return the contents of a file"""
    try:
        return filepath.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        try:
            return filepath.read_text(encoding="latin-1")
        except Exception:
            return "[Binary file or unable to read]"


def find_and_process_files():
    """
    Find all files in src and include directories (relative to repo root)
    and write concatenated output to kb/files_contents.txt
    """

    # Resolve repo root assuming this script lives in scripts/
    root = Path(__file__).resolve().parents[1]

    src_dirs = [
        root / "core" / "include",
        root / "core" / "src",
        root / "include",
        root / "src"
    ]

    kb_dir = root
    kb_dir.mkdir(exist_ok=True)

    output_path = kb_dir / "files_contents.txt"

    all_files: list[Path] = []

    for directory in src_dirs:
        if not directory.exists():
            print(f"Warning: Directory '{directory}' not found")
            continue

        for path in directory.rglob("*"):
            if not path.is_file():
                continue

            # Skip Eigen directories
            if "Eigen" in path.parts:
                continue

            all_files.append(path)

    # Sort for deterministic output
    all_files.sort(key=lambda p: str(p))

    with output_path.open("w", encoding="utf-8") as output_file:
        for filepath in all_files:
            rel_path = filepath.relative_to(root)

            output_file.write(f"{rel_path}\n")
            output_file.write("```\n")

            contents = read_file_contents(filepath)
            output_file.write(contents)

            if contents and not contents.endswith("\n"):
                output_file.write("\n")

            output_file.write("```\n\n")

    print(f"Successfully processed {len(all_files)} files")
    print(f"Output written to: {output_path}")


if __name__ == "__main__":
    find_and_process_files()
