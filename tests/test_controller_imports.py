import ast
from pathlib import Path


CONTROLLER_PATH = Path(__file__).resolve().parents[1] / "et_astlf_path_tracking" / "et_astlf_controller_node.py"


def test_controller_keeps_filesystem_path_and_ros_path_message_separate():
    tree = ast.parse(CONTROLLER_PATH.read_text(encoding="utf-8"))
    imports = [node for node in tree.body if isinstance(node, ast.ImportFrom)]

    pathlib_imports = [node for node in imports if node.module == "pathlib"]
    nav_imports = [node for node in imports if node.module == "nav_msgs.msg"]

    assert any(alias.name == "Path" and alias.asname == "FilePath" for node in pathlib_imports for alias in node.names)
    assert any(alias.name == "Path" and alias.asname == "PathMsg" for node in nav_imports for alias in node.names)
