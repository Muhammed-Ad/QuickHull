$repo = $null
if($args.count -eq 0 -or $null -eq $args[0] ){
    Write-Output "Script was not run with any input parameter"
    $repo = Read-Host "What is remote Repo name (case-sensitive) "
}
else {
    $repo = $args[0]
}


$yn = Read-Host "Are you sure this repo name is correct (y/n) " $repo

$command = "./createGitRepo.ps1 "
$ignore = "createGitRepo.ps1 `n*.exe `n*.out `n.gitignore `n/.vscode `n/`"old file from VS`" `n/`"project 1 testing`" `n*.zip"

if($yn -eq 'y'){
    Write-Output "# " $repo >> README.md
    Write-Output $ignore >> .gitignore
    $url = ("https://github.com/Muhammed-Ad/" + $repo + ".git")
    git init
    git add .gitignore README.md
    git commit -a -m "first commit"
    git branch -M main
    git remote add origin $url
    git push -u origin main

    git add .
    git commit -a -m "Update"
    git push
}
elseif($yn -eq 'n'){
    $name = Read-Host "Enter in correct name "
    $command += $name
    Invoke-Expression -Command $command -Verbose
}
else{
    Write-Output "Wrong input. Try again."
    Invoke-Expression -Command $command -Verbose
}