# Make arena_api importable from an interpreter that has a Windows Firewall inbound
# rule (the pixi Pythons on this PC do; a new venv's python.exe may not, and then
# camera discovery and streaming silently get nothing).
#
# Set ARENA_SITE_PACKAGES to the site-packages of a venv where these are installed:
#   pip install "C:\ProgramData\Lucid Vision Labs\ArenaView\ArenaPy\arena_api-2.9.5-py3-none-any.whl" "numpy<2"
# addsitedir (unlike PYTHONPATH) runs .pth files, which pywin32 (an arena_api dependency) needs.
import os
import site

path = os.environ.get('ARENA_SITE_PACKAGES')
if path:
    site.addsitedir(path)
