"""
Multi-Agent System - Custom Tools

Các tool hỗ trợ cho agents: syntax checking, logic analysis, etc.
"""

import subprocess
import ast


def check_python_syntax(code: str) -> dict:
    """Kiểm tra cú pháp Python bằng ast module."""
    try:
        ast.parse(code)
        return {"status": "OK", "errors": []}
    except SyntaxError as e:
        return {
            "status": "ERROR",
            "errors": [{"line": e.lineno, "message": str(e.msg)}],
        }


def check_cpp_syntax(filepath: str) -> dict:
    """Kiểm tra cú pháp C++ bằng g++ (chỉ check syntax, không compile)."""
    try:
        result = subprocess.run(
            ["g++", "-fsyntax-only", filepath],
            capture_output=True,
            text=True,
            timeout=30,
        )
        if result.returncode == 0:
            return {"status": "OK", "errors": []}
        else:
            return {"status": "ERROR", "errors": [result.stderr]}
    except FileNotFoundError:
        return {"status": "ERROR", "errors": ["g++ not found"]}
    except subprocess.TimeoutExpired:
        return {"status": "ERROR", "errors": ["Timeout"]}


def run_pylint(filepath: str) -> dict:
    """Chạy pylint để phân tích code quality."""
    try:
        result = subprocess.run(
            ["pylint", filepath, "--output-format=json"],
            capture_output=True,
            text=True,
            timeout=60,
        )
        return {"status": "OK", "output": result.stdout}
    except Exception as e:
        return {"status": "ERROR", "errors": [str(e)]}
