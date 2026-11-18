# Captura a area cliente da janela do visualizador Simbody (OpenSim) para um PNG.
# Uso: powershell -File capture_window.ps1 -out C:\caminho\frame.png [-front]
param([string]$out, [switch]$front, [string]$proc = "simbody-visualizer")
Add-Type -AssemblyName System.Drawing
Add-Type @"
using System; using System.Runtime.InteropServices;
public class W {
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr h, out R r);
  [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr h, ref P p);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr a, int x, int y, int cx, int cy, uint f);
  public struct R { public int L, T, Ri, B; }
  public struct P { public int X, Y; }
}
"@
[void][W]::SetProcessDPIAware()
$p = Get-Process $proc -ErrorAction Stop | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
$h = $p.MainWindowHandle
# -front: restaura, traz para frente e deixa SEMPRE POR CIMA (HWND_TOPMOST) durante a gravacao
if ($front) { [void][W]::ShowWindow($h, 9); [void][W]::SetWindowPos($h, [IntPtr](-1), 0, 0, 0, 0, 3); [void][W]::SetForegroundWindow($h); Start-Sleep -Milliseconds 300 }
$r = New-Object W+R; [void][W]::GetClientRect($h, [ref]$r)
$pt = New-Object W+P; [void][W]::ClientToScreen($h, [ref]$pt)
$w = $r.Ri - $r.L; $hh = $r.B - $r.T
$bmp = New-Object System.Drawing.Bitmap $w, $hh
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($pt.X, $pt.Y, 0, 0, $bmp.Size)
$bmp.Save($out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
"$w x $hh"
