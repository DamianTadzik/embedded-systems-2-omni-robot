import re

def strip_comments(code: str) -> str:
    code = re.sub(r'""".*?"""', '', code, flags=re.S)
    code = re.sub(r"'''.*?'''", '', code, flags=re.S)
    code = re.sub(r'#.*', '', code)
    lines = [line for line in code.splitlines() if line.strip() != ""]
    return "\n".join(lines)
