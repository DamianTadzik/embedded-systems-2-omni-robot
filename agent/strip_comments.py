import re

def strip_comments(code: str) -> str:
    # remove triple-quoted docstrings
    code = re.sub(r'""".*?"""', '', code, flags=re.S)
    code = re.sub(r"'''.*?'''", '', code, flags=re.S)

    # remove single line comments
    code = re.sub(r'#.*', '', code)

    # remove empty lines
    lines = [line for line in code.splitlines() if line.strip() != ""]
    return "\n".join(lines)
