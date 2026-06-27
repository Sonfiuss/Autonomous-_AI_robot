# coding: utf-8
"""
analyze_function.py
Given a class name, function name, and param list (as stored in functions.txt),
find the function body in the C++ source tree and return all method calls made
inside it, in the format:  CalledClass-MethodName(args)
The caller's type is resolved from:
  - parameters declared in the function signature
  - local variable declarations inside the body
Usage (CLI):
    python analyze_function.py --func "ClassName-FuncName(ParamType1, ParamType2)" \
                               --src  com/project/DBM \
                               --functions functions.txt
Usage (API):
    from analyze_function import analyze_function
    calls = analyze_function("DBMstdCodeMngCondition", "SetCodeMngCondition",
                             ["const DBMstdCodeMngCondition &"],
                             src_root="com/project/DBM")
"""
import os
import re
import argparse
# ---------------------------------------------------------------------------
# Patterns
# ---------------------------------------------------------------------------
# Detect  ClassName::FuncName(  start — params may be multiline
RE_FUNC_DEF = re.compile(
    r'^\s*(?:[\w:<>*&\s]+\s+)?'
    r'(\w+)'                        # class name  (group 1)
    r'::'
    r'(~?\w+)'                      # func name   (group 2)
    r'\s*\('
)
# Fallback: plain  FuncName(  with no class:: prefix
RE_FUNC_DEF_PLAIN = re.compile(
    r'^\s*(?:[\w:<>*&]+\s+)+'
    r'(~?\w+)'                      # func name only  (group 1)
    r'\s*\('
)
# Variable declaration:  Type name;  Type* name;  Type *name;  Type *&name = ...
RE_VAR_DECL = re.compile(
    r'^\s*([\w:<>]+)'       # base type  (group 1)
    r'[\s*&]+'              # separator: spaces/*/& in any combo
    r'(\w+)'                # variable name  (group 2)
    r'\s*(?:;|=|\(|\{)'
)
# Static call:  ClassName::StaticMethod(
RE_STATIC_CALL = re.compile(
    r'(?<![:\w])'
    r'([A-Z]\w+)'           # class name  (group 1)
    r'::'
    r'(\w+)'                # method name (group 2)
    r'\s*\('
)
# this->method(
RE_THIS_CALL = re.compile(
    r'\bthis\s*->\s*(\w+)\s*\('
)

# Function names to always ignore (helpers, macros, logging)
IGNORE_FUNCS = {
    'Format', '_T', 'LogError', 'MONITOR_ERROR0', 'AfxMessageBox',
    'TRACE', 'ASSERT', 'VERIFY', 'DEBUG_NEW', 'sizeof', 'alignof',
    # C++ keywords / operators that look like calls
    'if', 'for', 'while', 'do', 'switch', 'return', 'delete', 'throw',
    'catch', 'new', 'static_cast', 'dynamic_cast', 'reinterpret_cast',
    'const_cast', 'decltype', 'typeid', 'noexcept',
    # common STL free functions / names that are leaves
    'move', 'forward', 'make_shared', 'make_unique', 'make_pair',
    'make_tuple', 'get', 'tie', 'ref', 'cref', 'bind',
    'size', 'empty', 'begin', 'end', 'data', 'swap',
    'min', 'max', 'abs', 'sqrt', 'pow', 'floor', 'ceil',
    'printf', 'fprintf', 'sprintf', 'snprintf', 'memcpy', 'memset',
    'strlen', 'strcmp', 'strcpy', 'malloc', 'free', 'calloc',
    'assert', 'abort', 'exit',
}
# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _is_macro(name: str) -> bool:
    """ALL_CAPS_WITH_UNDERSCORES pattern = macro, not a real function."""
    return bool(re.match(r'^[A-Z][A-Z0-9_]+$', name))

def _effective_class(raw_type: str) -> str:
    """Extract the primary class name from a C++ type declaration.
    e.g. 'std::shared_ptr<orbit_base::ThreadPool>' -> 'ThreadPool'
         'DBMstdDatabase*'                          -> 'DBMstdDatabase'
         'uint32_t'                                 -> ''
    Strategy: collect all PascalCase/UpperCase tokens; take the last one,
    which is typically the actual class (not a namespace or wrapper).
    """
    names = re.findall(r'\b([A-Z]\w+)\b', raw_type)
    return names[-1] if names else ''


# member variable declaration inside a class body:
#   Type name_;  or  Type* name_ = default;  etc.
RE_MEMBER_DECL = re.compile(
    r'^\s*([\w:<>*&,\s]+?)'    # type (lazy)
    r'\s+(\w+_?)'               # name (may end with _)
    r'\s*(?:=|;|\{)'
)

SKIP_MEMBER_KEYWORDS = {
    'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'return',
    'delete', 'new', 'throw', 'catch', 'try', 'const', 'static',
    'virtual', 'explicit', 'inline', 'friend', 'override', 'mutable',
    'public', 'private', 'protected',
}

_member_cache: dict = {}   # class_name -> {var_name: effective_type}

def _build_member_typemap(class_name: str, src_root: str) -> dict:
    """Parse the header that declares `class class_name` and return
    {member_name: EffectiveClass} for all member variables."""
    cache_key = (class_name, src_root)
    if cache_key in _member_cache:
        return _member_cache[cache_key]

    result: dict = {}
    header = _find_class_header(class_name, src_root)
    if header is None:
        _member_cache[cache_key] = result
        return result

    try:
        with open(header, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError:
        _member_cache[cache_key] = result
        return result

    in_class = False
    brace_depth = 0
    class_start_depth = -1

    for line in lines:
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('*') or stripped.startswith('#'):
            brace_depth += line.count('{') - line.count('}')
            continue

        # Detect class entry
        if not in_class:
            cm = re.search(r'\bclass\s+' + re.escape(class_name) + r'\b', line)
            if cm and '{' in line:
                in_class = True
                class_start_depth = brace_depth + line.count('{')
                brace_depth += line.count('{') - line.count('}')
                continue

        if not in_class:
            brace_depth += line.count('{') - line.count('}')
            continue

        brace_depth += line.count('{') - line.count('}')

        # Exit class body
        if brace_depth < class_start_depth:
            break

        # Only look at direct class members (depth == class_start_depth)
        if brace_depth != class_start_depth:
            continue

        if stripped.startswith('//') or '(' in stripped:
            continue   # skip methods and comments

        m = RE_MEMBER_DECL.match(line)
        if not m:
            continue
        raw_type = m.group(1).strip()
        var_name = m.group(2).strip()
        if var_name in SKIP_MEMBER_KEYWORDS:
            continue
        eff = _effective_class(raw_type)
        if eff and eff != class_name:
            result[var_name] = eff

    _member_cache[cache_key] = result
    return result


def _find_class_header(class_name: str, src_root: str) -> str | None:
    """Walk src_root for the first .h/.hpp file containing 'class ClassName'."""
    pattern = re.compile(r'\bclass\s+' + re.escape(class_name) + r'\b')
    for dirpath, dirnames, filenames in os.walk(src_root):
        dirnames[:] = [d for d in dirnames
                       if d not in ('build', 'third_party', '.git', '__pycache__')]
        for fname in filenames:
            if not fname.endswith(('.h', '.hpp')):
                continue
            fpath = os.path.join(dirpath, fname)
            try:
                with open(fpath, encoding='utf-8', errors='replace') as fh:
                    if pattern.search(fh.read()):
                        return fpath
            except OSError:
                continue
    return None


def _clean_type(raw: str) -> str:
    """Strip const/pointer/ref noise to get the bare class name."""
    t = raw.replace('const', '').replace('*', '').replace('&', '').strip()
    # Remove leading :: if any
    return t.lstrip(':').strip()
def _parse_params_to_typemap(raw_params: str) -> dict:
    """
    'const DBMstdFoo &foo, short val'  ->  {'foo': 'DBMstdFoo', 'val': 'short'}
    Only registers params whose type starts with an uppercase letter (class types).
    """
    type_map = {}
    for param in raw_params.split(','):
        param = param.split('=')[0].strip()
        tokens = param.split()
        if len(tokens) < 2:
            continue
        var_name = tokens[-1].lstrip('*&').rstrip('[]').strip()
        raw_type = _clean_type(' '.join(tokens[:-1]))
        if var_name and re.match(r'^\w+$', var_name):
            type_map[var_name] = raw_type
    return type_map
# ---------------------------------------------------------------------------
# Core function
# ---------------------------------------------------------------------------
def analyze_function(class_name: str,
                     func_name: str,
                     param_types: list,
                     src_root: str,
                     functions_txt: str = '') -> list:
    """
    Find the definition of class_name::func_name in src_root, then return
    a list of method calls made inside the body:
        ['CalledClass-Method(args)', ...]
    param_types : list of type strings, e.g. ['const DBMstdFoo &', 'short']
                  (used only to match the correct overload when names collide)
    functions_txt : optional path to functions.txt produced by extract_functions.py
                    (reserved for future cross-reference; not required right now)
    """
    src_root = os.path.abspath(src_root)
    body_lines = _locate_function_body(class_name, func_name, src_root)
    if body_lines is None:
        return []
    signature_params, body = body_lines
    type_map = _parse_params_to_typemap(signature_params)
    type_map['this'] = class_name  # explicit this-> always resolves to owner class
    # Seed member variables from the class header so obj->method() on members resolves
    type_map.update(_build_member_typemap(class_name, src_root))

    calls = []

    # --- Pass 1: collect local variable declarations (line-anchored) ---
    for line in body:
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('*') or stripped.startswith('/*'):
            continue
        var_m = RE_VAR_DECL.match(line)
        if var_m:
            raw_type = _clean_type(var_m.group(1))
            var_name = var_m.group(2).strip()
            if re.match(r'^[A-Z]\w*$', raw_type) and re.match(r'^\w+$', var_name):
                type_map[var_name] = raw_type

    # --- Pass 2: scan joined body text for all call patterns ---
    text = ''.join(body)
    text = re.sub(r'//[^\n]*', ' ', text)           # strip line comments
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.DOTALL)  # strip block comments

    def _add(entry):
        if entry not in calls:
            calls.append(entry)

    # Pattern A — obj.method( / obj->method( / obj[i]->method(
    # Also picks up chained segments individually: a->b()->c( matches 'b' then 'c'
    RE_OBJ = re.compile(
        r'(\w+)'                    # object name
        r'(?:\[[^\]]*\])*'          # optional subscripts e.g. [i]
        r'\s*(?:->|\.)\s*'
        r'(\w+)'                    # method name
        r'\s*\('
    )
    for m in RE_OBJ.finditer(text):
        obj  = m.group(1)
        meth = m.group(2)
        if meth in IGNORE_FUNCS or _is_macro(meth):
            continue
        if obj == 'this':
            _add(f'{class_name}-{meth}()')
            continue
        resolved = type_map.get(obj)
        if resolved:
            _add(f'{resolved}-{meth}()')

    # Pattern B — (multi-level) namespace/class static call:  A::B::C(  or  Cls::Method(
    RE_STATIC = re.compile(
        r'(?<![:\w])([A-Z]\w+(?:::\w+)*)::(\w+)\s*\('
    )
    for m in RE_STATIC.finditer(text):
        cls  = m.group(1).split('::')[-1]
        meth = m.group(2)
        if meth not in IGNORE_FUNCS and not _is_macro(meth):
            _add(f'{cls}-{meth}()')

    # Pattern C — direct call with no preceding . -> ::  (any case)
    scrubbed = RE_OBJ.sub(' ', RE_STATIC.sub(' ', text))

    # Pattern D — new ClassName(  constructor
    RE_NEW = re.compile(r'\bnew\s+(\w+)\s*\(')
    for m in RE_NEW.finditer(scrubbed):
        cls = m.group(1)
        if cls not in IGNORE_FUNCS and not _is_macro(cls):
            _add(f'{cls}-{cls}()')

    # Pattern E — template call:  Func<Type>(  or  Func<A,B>(
    RE_TEMPLATE = re.compile(r'(?<![.>:\w])([A-Za-z_]\w*)\s*<[^>()]*>\s*\(')
    for m in RE_TEMPLATE.finditer(scrubbed):
        meth = m.group(1)
        if meth not in IGNORE_FUNCS and not _is_macro(meth):
            _add(f'{class_name}-{meth}()')
    scrubbed = RE_TEMPLATE.sub(' ', scrubbed)

    # Pattern F — chained return-value call:  expr()->method(  or  expr().method(
    RE_CHAIN = re.compile(r'\)\s*(?:->|\.)\s*(\w+)\s*\(')
    for m in RE_CHAIN.finditer(text):
        meth = m.group(1)
        if meth not in IGNORE_FUNCS and not _is_macro(meth):
            _add(f'-{meth}()')
    scrubbed = RE_CHAIN.sub(' ', scrubbed)

    RE_DIRECT = re.compile(r'(?<![.>:\w])([A-Za-z_]\w*)\s*\(')
    for m in RE_DIRECT.finditer(scrubbed):
        meth = m.group(1)
        if meth not in IGNORE_FUNCS and not _is_macro(meth):
            _add(f'{class_name}-{meth}()')

    return calls
# ---------------------------------------------------------------------------
# File locator — walk src_root to find ClassName::FuncName definition
# ---------------------------------------------------------------------------
def _locate_function_body(class_name: str, func_name: str, src_root: str):
    """
    Return (signature_params_str, [body lines]) for the first matching
    definition found under src_root, or None if not found.
    """
    for dirpath, dirnames, filenames in os.walk(src_root):
        dirnames[:] = [d for d in dirnames
                       if d not in ('build', 'third_party', '.git', '__pycache__')]
        for fname in filenames:
            if not fname.endswith(('.cpp', '.cc', '.cxx')):
                continue
            fpath = os.path.join(dirpath, fname)
            result = _extract_body_from_file(class_name, func_name, fpath)
            if result is not None:
                return result
    return None
def _collect_params_multiline(lines: list, start_line: int, open_col: int) -> tuple:
    """Scan from open_col on start_line, collect chars until matching ')'.
    Returns (params_str, end_line_index)."""
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
                buf.append(ch)
            else:
                buf.append(ch)
    return ''.join(buf), len(lines) - 1


def _extract_body_from_file(class_name: str, func_name: str, filepath: str):
    try:
        with open(filepath, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except OSError:
        return None

    n = len(lines)
    i = 0
    while i < n:
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('#') \
                or stripped.startswith('/*') or stripped.startswith('*'):
            i += 1
            continue

        m = RE_FUNC_DEF.match(line)
        matched = (m and m.group(1) == class_name
                   and m.group(2).lstrip('~') == func_name.lstrip('~'))
        if not matched:
            m2 = RE_FUNC_DEF_PLAIN.match(line)
            matched = (m2 and m2.group(1).lstrip('~') == func_name.lstrip('~')
                       and '::' not in line[:line.index('(')])
            if matched:
                m = m2  # reuse m for open_col below
        if matched:
            open_col = line.index('(', m.start())
            sig_params, param_end_line = _collect_params_multiline(lines, i, open_col)

            # Find opening brace of body (after the closing paren)
            brace_depth = 0
            body = []
            found_open = False
            j = param_end_line
            while j < n:
                l = lines[j]
                for ci, ch in enumerate(l):
                    if ch == '{' and not found_open:
                        found_open = True
                        # body starts after this brace
                        body.append(l[ci + 1:])
                        brace_depth = 1
                        break
                if found_open:
                    if j > param_end_line or (j == param_end_line and body):
                        # count braces in rest of line already appended
                        seg = body[-1] if body else ''
                        # re-count brace depth from the segment added
                        brace_depth += seg.count('{') - seg.count('}')
                    j += 1
                    break
                j += 1

            # Collect remaining body lines
            while j < n and brace_depth > 0:
                l = lines[j]
                brace_depth += l.count('{') - l.count('}')
                body.append(l)
                j += 1

            if found_open:
                return (sig_params, body)

        i += 1
    return None
# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_func_string(s: str):
    """
    Parse 'ClassName-FuncName(ParamType1, ParamType2)' into components.
    Returns (class_name, func_name, [param_types]).
    """
    m = re.match(r'^(\w+)-(\w+)\(([^)]*)\)$', s.strip())
    if not m:
        raise ValueError(f'Cannot parse function string: {s!r}')
    class_name = m.group(1)
    func_name  = m.group(2)
    params     = [p.strip() for p in m.group(3).split(',') if p.strip()]
    return class_name, func_name, params
def main():
    parser = argparse.ArgumentParser(
        description='Analyze calls made inside a specific C++ method.')
    parser.add_argument('--func', required=True,
                        help='Function to analyze, e.g. "ClassName-FuncName(Type1,Type2)"')
    parser.add_argument('--src', required=True,
                        help='Root folder of C++ source files')
    parser.add_argument('--functions', default='',
                        help='Path to functions.txt (optional, for future cross-ref)')
    parser.add_argument('--out', default='',
                        help='Write output to file instead of stdout')
    args = parser.parse_args()
    class_name, func_name, param_types = _parse_func_string(args.func)
    calls = analyze_function(
        class_name=class_name,
        func_name=func_name,
        param_types=param_types,
        src_root=args.src,
        functions_txt=args.functions,
    )
    if not calls:
        print(f'[analyze_function] No resolvable calls found in '
              f'{class_name}::{func_name}')
        return
    output = '\n'.join(calls)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            fh.write(output + '\n')
        print(f'[analyze_function] {len(calls)} calls written to {args.out}')
    else:
        print(output)
if __name__ == '__main__':
    main()
