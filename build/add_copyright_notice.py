"""Script to add a copyright notice to all .cpp and .h files in a specified directory."""

import os

NOTICE = """// A minimalistic 2D physics engine (https://github.com/dmitry-sapelnikov/neat-physics)
// SPDX-FileCopyrightText: 2025 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

"""


def add_notice_to_cpp_files(
    root_dir: str, files_to_ignore: list[str], directories_to_ignore: list[str]
) -> None:
    """Adds a copyright notice to all .cpp and .h files in the specified directory.

    Args:
        root_dir (str): The root directory to search for files.
        files_to_ignore (list): Set of file names to ignore.
        directories_to_ignore (list): Set of directory names to ignore.
    """
    for dirpath, _, filenames in os.walk(root_dir):
        if any(ignored in dirpath for ignored in directories_to_ignore):
            continue

        for filename in filenames:
            if filename.endswith(".cpp") or filename.endswith(".h"):
                if any(ignored in filename for ignored in files_to_ignore):
                    continue

                file_path = os.path.join(dirpath, filename)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                if not content.startswith(NOTICE):
                    with open(file_path, "w", encoding="utf-8") as f:
                        f.write(NOTICE + content)
                    print(f"Notice added to: {file_path}")


def main() -> None:
    """Main function to execute the script."""
    this_script_dir = os.path.dirname(os.path.abspath(__file__))
    target_directory = os.path.join(this_script_dir, "..")

    directories_to_ignore = ["3rd_party", "KHR", "imgui"]
    files_to_ignore = ["glad", "stb_image"]
    add_notice_to_cpp_files(target_directory, files_to_ignore, directories_to_ignore)


if __name__ == "__main__":
    main()
