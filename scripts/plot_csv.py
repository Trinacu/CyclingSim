import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import argparse


def plot_csv(csv_path):
    """
    Plot data from a CSV file.

    Parameters:
    -----------
    csv_path : str
        Path to the CSV file
    """

    # Check if file exists
    if not os.path.exists(csv_path):
        print(f"Error: File '{csv_path}' not found!")
        print(f"Current working directory: {os.getcwd()}")
        return

    try:
        # Read CSV file
        df = pd.read_csv(csv_path)
        print(f"Successfully loaded '{csv_path}'")
        print(f"Columns: {list(df.columns)}")
        print(f"Shape: {df.shape}")

        # Create plot
        plt.figure(figsize=(10, 6))

        # Plot all columns (first column as x, others as y)
        x = df.iloc[:, 0]

        if len(df.columns) == 2:
            # Single y column
            y = df.iloc[:, 1]
            plt.plot(x, y, 'b-', linewidth=2, label=df.columns[1])
        else:
            # Multiple y columns
            for i in range(1, len(df.columns)):
                y = df.iloc[:, i]
                plt.plot(x, y, linewidth=2, label=df.columns[i])
            plt.legend()

        # Customize plot
        filename = os.path.basename(csv_path)
        plt.title(f'Plot: {filename}', fontsize=14, fontweight='bold')
        plt.xlabel(df.columns[0], fontsize=12)
        plt.ylabel('Value', fontsize=12)
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.tight_layout()

        # Save plot
        base_name = os.path.splitext(filename)[0]
        plot_path = f"{base_name}_plot.png"
        plt.savefig(plot_path, dpi=150)
        print(f"Plot saved as '{plot_path}'")

        # Show plot
        plt.show()

    except Exception as e:
        print(f"Error reading or plotting data: {e}")


def main():
    parser = argparse.ArgumentParser(description='Plot data from CSV file')
    parser.add_argument('csv_file', help='Path to CSV file')
    args = parser.parse_args()

    plot_csv(args.csv_file)


if __name__ == "__main__":
    main()
