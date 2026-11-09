"""
Automated Cross-Platform LaTeX Project Report Compiler (Windows, macOS, Linux)
- Checks for existing LaTeX compilers (tectonic, pdflatex, xelatex).
- Uses standalone Tectonic compiler from system PATH or local bin.
- Compiles main.tex to PDF with two-pass resolution for TOC, LOF, LOT, and cross-references.
"""

import os
import sys
import subprocess
import shutil
import platform

def find_compiler(target_dir):
    # 1. Check system PATH
    for cmd in ["tectonic", "pdflatex", "xelatex"]:
        path = shutil.which(cmd)
        if path:
            print(f"[+] Found system LaTeX compiler: {path}")
            return path

    # 2. Check local directories
    candidates = [
        os.path.join(r"d:\DEV\Final Year Project\Latex\bin", "tectonic.exe"),
        os.path.join(r"d:\DEV\Final Year Project\ProjectReport\bin", "tectonic.exe"),
        os.path.join(target_dir, "bin", "tectonic.exe"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate) and os.path.isfile(candidate):
            print(f"[+] Found local Tectonic compiler: {candidate}")
            return candidate

    print("[-] Error: No LaTeX compiler found!")
    sys.exit(1)

def compile_report(main_tex_path, out_dir=None):
    if not os.path.exists(main_tex_path):
        print(f"[-] Error: File not found: {main_tex_path}")
        sys.exit(1)

    work_dir = os.path.dirname(os.path.abspath(main_tex_path))
    if out_dir is None:
        out_dir = work_dir
    os.makedirs(out_dir, exist_ok=True)

    compiler = find_compiler(work_dir)
    print(f"[*] Compiling {main_tex_path} using {compiler}...")

    main_basename = os.path.basename(main_tex_path)
    cmd_name = os.path.basename(compiler).lower()
    if "tectonic" in cmd_name:
        cmd = [compiler, "-X", "compile", main_basename, "--outdir", out_dir]
    else:
        cmd = [compiler, "-interaction=nonstopmode", f"-output-directory={out_dir}", main_basename]

    print(f"[*] Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=work_dir, capture_output=True, text=True)

    if result.returncode != 0:
        print("[-] Compilation failed!")
        print(result.stdout[-1500:])
        print(result.stderr[-1500:])
        sys.exit(1)

    # Second pass for references / TOC
    print("[*] Running second pass for cross-references & TOC...")
    result = subprocess.run(cmd, cwd=work_dir, capture_output=True, text=True)

    pdf_name = os.path.splitext(os.path.basename(main_tex_path))[0] + ".pdf"
    pdf_path = os.path.join(out_dir, pdf_name)
    if os.path.exists(pdf_path):
        print(f"[+] Success! Output PDF generated at: {pdf_path}")
        print(f"    Size: {os.path.getsize(pdf_path) / 1024:.1f} KB")
        return pdf_path
    else:
        print("[-] PDF file not found after compilation.")
        sys.exit(1)

if __name__ == "__main__":
    tex_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "main.tex")
    compile_report(tex_file)
