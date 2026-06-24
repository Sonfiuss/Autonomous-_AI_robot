# coding: utf-8
"""
extract_functions.py
Scan C++ source files and extract function signatures.
Output format per function:
    <ClassName>-<FunctionName>(<ParamType1>,<ParamType2>,...)
Usage:
    python extract_functions.py <root_folder> [--out output.txt]
"""
import os
import re
import argparse
# Detect start of  ClassName::FuncName(  (params may continue on next lines)
RE_IMPL_START = re.compile(
    r'^\s*'
    r'(?:[\w:<>*&\s]+\s+)?'        # optional return type
    r'(\w+)'                        # class name  (group 1)
    r'::'
    r'(\w+|operator\s*[^\(]+)'     # function name (group 2)
    r'\s*\('
)
# Fallback: plain static/free function  e.g. static void SleepFor2Ms(
RE_PLAIN_FUNC = re.compile(
    r'^\s*(?:static\s+|inline\s+|ORBIT_NOINLINE\s+)*'
    r'(?:[\w:<>*&]+\s+)+'          # return type (one or more tokens)
    r'(\w+)'                        # function name  (group 1)
    r'\s*\('
)
# Match class declaration in .h:  class ClassName  or  class ClassName :
RE_CLASS_DECL = re.compile(r'^\s*(?:class|struct)\s+(\w+)\s*(?:[:,{]|$)')
# Match method declaration inside class body in .h:
#   ReturnType FunctionName(...)
#   FunctionName(...)   <- constructor
RE_METHOD_DECL = re.compile(
    r'^\s*'
    r'(?:(?:virtual|static|inline|explicit|friend|override|const)\s+)*'
    r'(?:[\w:<>*&]+\s+)*'          # return type tokens
    r'(\w+)'                        # function name
    r'\s*\('
    r'([^)]*)'                      # param list
    r'\)\s*(?:const\s*)?(?:override\s*)?(?:=\s*0\s*)?[;{]'
)
SKIP_KEYWORDS = {
    'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'return',
    'delete', 'new', 'throw', 'catch', 'try', 'sizeof', 'alignof',
    'decltype', 'static_assert', 'namespace', 'class', 'struct',
    'enum', 'union', 'typedef', 'using', 'nullptr', 'true', 'false',
    'public', 'private', 'protected', 'friend', 'template',
}
def extract_param_types(raw: str) -> str:
    """Keep only type names from param list, drop variable names and defaults."""
    raw = raw.strip()
    if not raw:
        return ''
    parts = []
    for param in raw.split(','):
        param = param.split('=')[0].strip()   # drop default value
        # Remove array suffix e.g. "int arr[]"
        param = re.sub(r'\[.*?\]', '', param).strip()
        tokens = param.split()
        if not tokens:
            continue
        # Last token is the variable name if it looks like an identifier
        # and there is at least one type token before it.
        if len(tokens) > 1 and re.match(r'^[a-z_]\w*$', tokens[-1]):
            tokens = tokens[:-1]
        # Join type tokens (e.g. "const short" -> "constshort", "short*" -> "short*")
        parts.append(' '.join(tokens))
    return ', '.join(parts)
def _collect_params(lines: list, start_line: int, open_col: int) -> tuple:
    """
    Starting from open_col (the '(' column) on start_line,
    scan forward counting paren depth until matching ')'.
    Return (params_str, end_line_index).
    """
    depth = 0
    buf = []
    for li in range(start_line, len(lines)):
        segment = lines[li] if li > start_line else lines[li][open_col:]
        for ch in segment:
            if ch == '(':
                depth += 1
                if depth > 1:
                    buf.append(ch)
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    return ''.join(buf), li
                else:
                    buf.append(ch)
            else:
                buf.append(ch)
    return ''.join(buf), len(lines) - 1


def parse_cpp(filepath: str):
    """Extract from .cpp: ClassName::FunctionName(params), handles multiline params."""
    results = []
    try:
        with open(filepath, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError:
        return results

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('*') \
                or stripped.startswith('#') or stripped.startswith('/*'):
            i += 1
            continue

        m = RE_IMPL_START.match(line)
        if m:
            class_name = m.group(1)
            func_name  = m.group(2).strip()
        else:
            # Try plain static/free function (no ClassName:: prefix)
            paren_idx = line.find('(')
            if paren_idx == -1 or '::' in line[:paren_idx]:
                i += 1
                continue
            m2 = RE_PLAIN_FUNC.match(line)
            if not m2:
                i += 1
                continue
            class_name = ''
            func_name  = m2.group(1).strip()
            m = m2

        if func_name in SKIP_KEYWORDS or class_name in SKIP_KEYWORDS:
            i += 1
            continue

        # Find opening paren position and collect params (possibly multiline)
        open_col = line.index('(', m.start())
        raw_params, end_line = _collect_params(lines, i, open_col)

        func_name = func_name.lstrip('~') or ('~' + class_name)
        param_str = extract_param_types(raw_params)
        results.append(f'{class_name}-{func_name}({param_str})')

        i = end_line + 1
    return results
def parse_header(filepath: str):
    """Extract from .h: detect class scope then method declarations."""
    results = []
    try:
        with open(filepath, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError:
        return results
    # Simple brace-depth class scope tracker
    class_stack  = []   # list of (class_name, entry_depth)
    brace_depth  = 0
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('*') \
                or stripped.startswith('#') or stripped.startswith('/*'):
            brace_depth += line.count('{') - line.count('}')
            continue
        open_n  = line.count('{')
        close_n = line.count('}')
        cls_m = RE_CLASS_DECL.match(line)
        if cls_m and open_n > 0:
            class_stack.append((cls_m.group(1), brace_depth + open_n - close_n))
        brace_depth += open_n - close_n
        # Pop closed classes
        class_stack = [(n, d) for n, d in class_stack if d <= brace_depth]
        if cls_m:
            continue
        if not class_stack:
            continue
        current_class = class_stack[-1][0]
        m = RE_METHOD_DECL.match(line)
        if not m:
            continue
        func_name  = m.group(1).strip()
        raw_params = m.group(2)
        if func_name in SKIP_KEYWORDS:
            continue
        param_str = extract_param_types(raw_params)
        results.append(f'{current_class}-{func_name}({param_str})')
    return results
# ---------------------------------------------------------------------------
# Call extractor — finds method calls inside each function body
# ---------------------------------------------------------------------------
# Variable declaration:  TypeName varName;  or  TypeName* varName;  or  TypeName varName(
# Also handles:  TypeName varName = ...
RE_VAR_DECL = re.compile(
    r'^\s*'
    r'(\w[\w:<>*&\s]*?)'       # type (group 1) — lazy so it stops before name
    r'\s+(\w+)'                 # variable name (group 2)
    r'\s*(?:;|=|\(|{)'
)
# Method call on object:  varName.methodName(args)  or  varName->methodName(args)
RE_METHOD_CALL = re.compile(
    r'(\w+)'                    # object / variable name (group 1)
    r'(?:\.|-&gt;|->)'         # . or ->
    r'(\w+)'                    # method name (group 2)
    r'\s*\(([^)]*)\)'           # args (group 3)
)
# Function signature start in .cpp:  ClassName::FunctionName(
RE_FUNC_START = re.compile(
    r'^\s*(?:[\w:<>*&\s]+\s+)?(\w+)::(\w+)\s*\([^)]*\)\s*(?:const\s*)?(?::\s*\w.*)?$'
)
def extract_calls_in_functions(filepath: str) -> dict:
    """
    Parse a .cpp file and return a dict:
        key   -> "OwnerClass-FunctionName"
        value -> list of "CalledClass-CalledMethod(args)" strings
    Strategy:
    - Track function bodies by watching ClassName::FuncName() then { ... }
    - Inside each body, collect variable declarations to build a type map
    - Match obj.method(args) or obj->method(args) and resolve type from map
    - Params of the outer function are also added to the type map
    """
    try:
        with open(filepath, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError:
        return {}
    results = {}
    # State
    in_body       = False
    brace_depth   = 0
    body_start_depth = 0
    current_key   = None
    type_map      = {}   # varname -> typename (within current function)
    calls         = []
    # Parse param list of current function to pre-populate type_map
    def register_params(raw_params: str):
        for param in raw_params.split(','):
            param = param.split('=')[0].strip()
            tokens = param.split()
            if len(tokens) >= 2:
                # last token = name, everything before = type
                var_name = tokens[-1].lstrip('*&').rstrip('[]')
                var_type = ' '.join(tokens[:-1]).replace('*','').replace('&','').strip()
                if var_name and re.match(r'^\w+$', var_name):
                    type_map[var_name] = var_type
    for line in lines:
        stripped = line.strip()
        # Skip comments / preprocessor
        if stripped.startswith('//') or stripped.startswith('*') \
                or stripped.startswith('#') or stripped.startswith('/*'):
            continue
        open_n  = line.count('{')
        close_n = line.count('}')
        if not in_body:
            # Look for function signature line (no brace yet, or brace on same line)
            sig_m = RE_FUNC_START.match(stripped)
            if sig_m:
                class_name = sig_m.group(1)
                func_name  = sig_m.group(2)
                current_key = f'{class_name}-{func_name}'
                type_map = {}
                calls = []
                # Extract params from this line to seed type_map
                paren_open  = line.find('(')
                paren_close = line.rfind(')')
                if paren_open != -1 and paren_close != -1:
                    register_params(line[paren_open+1:paren_close])
            if open_n > 0 and current_key:
                in_body = True
                body_start_depth = brace_depth + 1  # depth inside the function body
        else:
            # Inside function body — track variable declarations
            var_m = RE_VAR_DECL.match(line)
            if var_m:
                raw_type = var_m.group(1).replace('const','').replace('*','').replace('&','').strip()
                var_name = var_m.group(2).strip()
                # Only register if type looks like a class name (starts uppercase)
                if re.match(r'^[A-Z]\w*$', raw_type) and re.match(r'^\w+$', var_name):
                    type_map[var_name] = raw_type
            # Find all method calls on this line
            for call_m in RE_METHOD_CALL.finditer(line):
                obj_name  = call_m.group(1)
                meth_name = call_m.group(2)
                raw_args  = call_m.group(3).strip()
                resolved_type = type_map.get(obj_name)
                if resolved_type:
                    calls.append(f'{resolved_type}-{meth_name}({raw_args})')
        brace_depth += open_n - close_n
        # Detect end of function body
        if in_body and brace_depth < body_start_depth:
            in_body = False
            if current_key and calls:
                results[current_key] = list(dict.fromkeys(calls))  # deduplicate, keep order
            current_key = None
    return results
def scan_folder(root: str):
    """Return (function_list, call_graph).
    function_list : ['ClassName-FuncName(ParamTypes)', ...]
    call_graph    : {'ClassName-FuncName': ['CalledClass-Method(args)', ...], ...}
    """
    root = os.path.abspath(root)
    seen_funcs = set()
    function_list = []
    call_graph = {}
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if d not in ('build', 'third_party', '.git', '__pycache__')]
        for fname in sorted(filenames):
            ext = os.path.splitext(fname)[1].lower()
            fpath = os.path.join(dirpath, fname)
            if ext in ('.cpp', '.cc', '.cxx'):
                entries = parse_cpp(fpath)
                calls   = extract_calls_in_functions(fpath)
                call_graph.update(calls)
            elif ext in ('.h', '.hpp'):
                entries = parse_header(fpath)
                calls   = {}
            else:
                continue
            for e in entries:
                if e not in seen_funcs:
                    seen_funcs.add(e)
                    function_list.append(e)
    return function_list, call_graph
def main():
    parser = argparse.ArgumentParser(description='Extract C++ class methods and call graph.')
    parser.add_argument('folder', help='Root folder to scan')
    parser.add_argument('--out', default='', help='Output file (default: stdout)')
    parser.add_argument('--calls', action='store_true', help='Show call graph instead of function list')
    args = parser.parse_args()
    function_list, call_graph = scan_folder(args.folder)
    if args.calls:
        lines = []
        for caller, callees in sorted(call_graph.items()):
            for callee in callees:
                lines.append(f'{caller} -> {callee}')
        output = '\n'.join(lines)
    else:
        output = '\n'.join(function_list)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            fh.write(output + '\n')
        print(f'[extract_functions] written to {args.out}')
    else:
        print(output)
if __name__ == '__main__':
    main()
