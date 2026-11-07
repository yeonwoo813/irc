"""Read depth controls and save them without rewriting the rest of the INI."""

import configparser
import math
import os
from pathlib import Path
import re
import tempfile


SECTION = 'realsense_depth'
DEFAULTS = {
    'enable_auto_exposure': False,
    'exposure': 5000,
    'gain': 16,
    'laser_power': 150.0,
}
PARAMETER_NAMES = tuple('depth_module.' + key for key in DEFAULTS)


def _parser(text):
    parser = configparser.ConfigParser(
        interpolation=None, inline_comment_prefixes=('#', ';'),
    )
    parser.read_string(text)
    return parser


def validate_settings(values):
    """Check types/basic bounds; the camera checks its own hardware limits."""
    values = {key: values[key] for key in DEFAULTS}
    if type(values['enable_auto_exposure']) is not bool:
        raise ValueError('enable_auto_exposure must be true or false')
    for key in ('exposure', 'gain'):
        if type(values[key]) is not int:
            raise ValueError(f'{key} must be an integer')
    if values['exposure'] <= 0 or values['gain'] < 0:
        raise ValueError('exposure must be positive and gain must be nonnegative')
    laser = values['laser_power']
    if isinstance(laser, bool) or not isinstance(laser, (int, float)):
        raise ValueError('laser_power must be a number')
    if not math.isfinite(laser) or laser < 0:
        raise ValueError('laser_power must be finite and nonnegative')
    values['laser_power'] = float(laser)
    return values


def load_depth_settings(path):
    parser = _parser(Path(path).expanduser().read_text(encoding='utf-8'))
    getters = {'enable_auto_exposure': parser.getboolean,
               'exposure': parser.getint, 'gain': parser.getint,
               'laser_power': parser.getfloat}
    return validate_settings({
        key: getters[key](SECTION, key, fallback=default)
        for key, default in DEFAULTS.items()
    })


def depth_parameters(path):
    return {'depth_module.' + key: value
            for key, value in load_depth_settings(path).items()}


def load_depth_display_settings(path):
    parser = _parser(Path(path).expanduser().read_text(encoding='utf-8'))
    result = {key: parser.getfloat('realsense_yolo', key, fallback=default)
              for key, default in [('depth_scale', 0.001),
                                   ('depth_min_m', 0.08), ('depth_max_m', 3.0)]}
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError('Depth display settings must be finite')
    if result['depth_scale'] <= 0 or not (
            0 <= result['depth_min_m'] < result['depth_max_m']):
        raise ValueError('Invalid depth scale or display range')
    return result


def save_depth_settings(path, values):
    """Atomically update only our four keys, preserving comments/other edits."""
    values = validate_settings(values)
    path = Path(path).expanduser().resolve(strict=True)
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
        if match.group(1) == SECTION:
            start = i + 1
    if start is None:
        if lines and not lines[-1].endswith('\n'):
            lines[-1] += '\n'
        lines.extend(['\n', f'[{SECTION}]\n'])
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
