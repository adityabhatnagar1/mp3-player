$out = 'project.md'
$root = (Get-Location).Path
$skipDirs = '\\(\.git|__pycache__|node_modules|\.venv|build|\.vs)(\\|$)'
$binExt = '.mp3','.wav','.png','.jpg','.jpeg','.gif','.zip','.exe','.bin','.pyc','.pdf','.ico'
$fence = '````'
$lang = @{ '.cpp'='cpp'; '.h'='cpp'; '.ino'='cpp'; '.py'='python'; '.sh'='bash'; '.md'='markdown'; '.json'='json'; '.ps1'='powershell' }

$files = Get-ChildItem -Recurse -File -Force |
  Where-Object { $_.FullName -notmatch $skipDirs -and $_.Name -ne $out } |
  Sort-Object FullName

$tree = if (Get-Command eza -ErrorAction SilentlyContinue) { eza --tree -I '__pycache__|node_modules|build' } else { tree /F /A }

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine('# Project Tree').AppendLine().AppendLine($fence + 'text')
[void]$sb.AppendLine(($tree -join "`n")).AppendLine($fence).AppendLine()

foreach ($f in $files) {
  $rel = $f.FullName.Substring($root.Length + 1) -replace '\\','/'
  [void]$sb.AppendLine("### $rel").AppendLine()
  if ($binExt -contains $f.Extension.ToLower()) {
    [void]$sb.AppendLine('_(binary file, content omitted)_').AppendLine()
    } else {
    $l = $lang[$f.Extension.ToLower()]
    $content = Get-Content -Raw -LiteralPath $f.FullName
    if ($null -eq $content) { $content = '' }
    [void]$sb.AppendLine($fence + $l)
    [void]$sb.AppendLine($content.TrimEnd())
    [void]$sb.AppendLine($fence).AppendLine()
  }
}
$sb.ToString() | Set-Content -Path $out -Encoding UTF8
"Wrote $out ($($files.Count) files)"