"""Serialize atomic INI section updates from independent camera windows."""

import configparser
import fcntl
import os
from pathlib import Path
import re
import tempfile


def _parser(text):
    parser = configparser.ConfigParser(
        interpolation=None, inline_comment_prefixes=('#', ';'),
    )
    parser.read_string(text)
    return parser


def save_ini_section(path, section, values):
    path = Path(path).expanduser().resolve(strict=True)
    # Lock a stable sibling: replacing the INI changes its inode.
    with path.with_name('.' + path.name + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        _save_section(path, section, values)


def _save_section(path, section, values):
    original = path.read_text(encoding='utf-8')
    _parser(original)  # Refuse to overwrite a malformed or duplicate section.
    lines = original.splitlines(keepends=True)
    start = end = None
    for i, line in enumerate(lines):
        match = re.match(r'^\s*\[([^]]+)\]', line)
        if not match:
            continue
        if start is not None:
            end = i
            break
        if match.group(1) == section:
            start = i + 1
    if start is None:
        if lines and not lines[-1].endswith('\n'):
            lines[-1] += '\n'
        lines.extend(['\n', f'[{section}]\n'])
        start = len(lines)
    if end is None:
        end = len(lines)
    encoded = {key: str(value).lower() if isinstance(value, bool) else str(value)
               for key, value in values.items()}
    seen = set()
    for i in range(start, end):
        match = re.match(r'^(\s*([\w]+)\s*[=:]\s*)(.*?)(\r?\n)?$', lines[i])
        if not match or match.group(2).lower() not in encoded:
            continue
        key = match.group(2).lower()
        comment = re.search(r'\s+[;#].*$', match.group(3))
        suffix = comment.group(0) if comment else ''
        lines[i] = match.group(1) + encoded[key] + suffix + (match.group(4) or '\n')
        seen.add(key)
    missing = [f'{key} = {value}\n' for key, value in encoded.items() if key not in seen]
    if missing and end and not lines[end - 1].endswith('\n'):
        lines[end - 1] += '\n'
    lines[end:end] = missing
    updated = ''.join(lines)
    _parser(updated)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
