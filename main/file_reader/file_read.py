import os


def concatenate_files(directory, output_file):
    """
    Args:
        directory (str): Path to the directory containing the text files.
        output_file (str): Path to the output file where the concatenated
        content will be written.
    """

    with open(output_file, 'w') as outfile:
        for root, _, files in os.walk(directory):
            for filename in files:
                if not filename.endswith('.py'):  # Skip non-text files
                    continue

                filepath = os.path.join(root, filename)
                with open(filepath, 'r') as infile:
                    content = infile.read()

                # Create section header
                relative_path = os.path.relpath(filepath, directory)
                header = f"##### {relative_path}\n"
                outfile.write(header)
                outfile.write(content)
                outfile.write('\n')  # Add a newline after each section





def main():

    # Example usage
    directory = "/Users/seyed.akhavan/HosseinMac/local-old/tutorials-python/main/mpi-sppy"
    output_file = "./concatenated_files.txt"
    
    concatenate_files(directory, output_file)



if __name__ == '__main__':
    main()