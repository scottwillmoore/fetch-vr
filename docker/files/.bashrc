if [[ ! $- =~ i ]]; then
	return
fi

if [[ -f /etc/bash.bashrc ]]; then
	source "/etc/bash.bashrc"
fi

if [[ -f /etc/bash_completion ]]; then
	source "/etc/bash_completion"
fi

if [[ -f /opt/ros/ROS_DISTRIBUTION/setup.bash ]]; then
	source "/opt/ros/ROS_DISTRIBUTION/setup.bash"
fi

alias grep="grep --color=auto"
alias ls="ls --color=auto"
alias sudo="sudo "

export HISTFILESIZE=
export HISTSIZE=
export LESS="--raw-control-chars"
export LESS_TERMCAP_mb=$'\e[1;36m'
export LESS_TERMCAP_md=$'\e[1;36m'
export LESS_TERMCAP_me=$'\e[0m'
export LESS_TERMCAP_se=$'\e[0m'
export LESS_TERMCAP_so=$'\e[1;31m'
export LESS_TERMCAP_ue=$'\e[0m'
export LESS_TERMCAP_us=$'\e[1;35m'
export PS1="\w \$ "

shopt -s autocd
shopt -s autocd
shopt -s cdspell
shopt -s globstar
shopt -s histappend

if command -v dircolors &> /dev/null; then
	if [[ -f ~/.dircolors ]]; then
		eval "$(dircolors --bourne-shell ~/.dircolors)"
	else
		eval "$(dircolors --bourne-shell)"
	fi
fi

if command -v starship &> /dev/null; then
	eval "$(starship init bash)"
fi
