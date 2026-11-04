param([Parameter(Mandatory=$true)][int]$GamePid)
$ErrorActionPreference='Stop'
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class ProbeWindow {
 [StructLayout(LayoutKind.Sequential)] public struct Point { public int x,y; }
 [StructLayout(LayoutKind.Sequential)] public struct Rect { public int left,top,right,bottom; }
 [StructLayout(LayoutKind.Sequential)] public struct Placement { public int length,flags,showCmd; public Point min,max; public Rect normal; }
 [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
 [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
 [DllImport("user32.dll")] public static extern bool GetWindowPlacement(IntPtr h,ref Placement p);
 [DllImport("user32.dll")] public static extern bool SetWindowPlacement(IntPtr h,ref Placement p);
 [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h,IntPtr after,int x,int y,int w,int height,uint flags);
 [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h,out Rect r);
}
'@
$process=Get-Process -Id $GamePid
$window=$process.MainWindowHandle
if($window -eq [IntPtr]::Zero){throw 'Probe window is not ready'}
$before=[ProbeWindow]::GetForegroundWindow()
$placement=New-Object ProbeWindow+Placement
$placement.length=[Runtime.InteropServices.Marshal]::SizeOf($placement)
if(-not [ProbeWindow]::GetWindowPlacement($window,[ref]$placement)){throw 'GetWindowPlacement failed'}
$placement.showCmd=4 # SW_SHOWNOACTIVATE
$placement.normal.left=-30000;$placement.normal.top=-30000
$placement.normal.right=-29360;$placement.normal.bottom=-29552
if(-not [ProbeWindow]::SetWindowPlacement($window,[ref]$placement)){throw 'SetWindowPlacement failed'}
if(-not [ProbeWindow]::SetWindowPos($window,[IntPtr]::Zero,-30000,-30000,640,448,0x14)){throw 'SetWindowPos failed'} # NOACTIVATE | NOZORDER
$bounds=New-Object ProbeWindow+Rect
if(-not [ProbeWindow]::GetWindowRect($window,[ref]$bounds)){throw 'GetWindowRect failed'}
if($bounds.left -gt -10000 -or $bounds.top -gt -10000){throw 'Probe window is not off-screen'}
$after=[ProbeWindow]::GetForegroundWindow()
[uint32]$beforeProcess=0;[uint32]$afterProcess=0
$null=[ProbeWindow]::GetWindowThreadProcessId($before,[ref]$beforeProcess)
$null=[ProbeWindow]::GetWindowThreadProcessId($after,[ref]$afterProcess)
"Probe bounds=$($bounds.left),$($bounds.top); foreground before=$beforeProcess after=$afterProcess; probe=$GamePid"
if($beforeProcess -eq $GamePid -or $afterProcess -eq $GamePid){throw 'Probe took focus'}
if($after -ne $before){'Foreground changed between other applications; probe did not become foreground'}
