#!/usr/bin/env python3
import configparser
import subprocess

myRepHttps = 'https://github.com/TSUSAKA-ucl/'
myRepSsh = 'git@github.com:TSUSAKA-ucl/'
config = configparser.ConfigParser()
config.read('.gitmodules')
paths = list(map(lambda x: config[x]['path'], config.sections()))
urls = list(map(lambda x: config[x]['url'], config.sections()))
for path, url in zip(paths, urls):
    if url.startswith(myRepHttps):
        submod = url[len(myRepHttps):]
        if not submod.endswith('.git'):
            submod += '.git'
        sshUrl = myRepSsh + submod
        try:
            subprocess.run(['git', 'ls-remote', sshUrl], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f'change to SSH access. {submod}')
            subprocess.run(['git', 'config', f'submodule.{path}.url', sshUrl],
                           check=True)
            print(f'updated submodules for {submod}')
        except subprocess.CalledProcessError:
            print(f'cannot access {submod} using SSH')
# #!/usr/bin/bash
# for submod in moveit_jacobian gjk_worker
# do URL="git@github.com:TSUSAKA-ucl/$submod".git
#    if git ls-remote "$URL" 1>/dev/null 2>/dev/null
#    then echo "change to SSH access. $submod"
# 	git config submodule.wasm/"$submod".url "$URL"
#    else echo "cannot access $submod using SSH"
#    fi
# done
