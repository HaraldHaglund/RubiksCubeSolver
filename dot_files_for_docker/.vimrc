"ricing
set encoding=utf-8
set tabstop=4
set number
set relativenumber
set nocompatible
syntax enable
filetype plugin on
" add <> matching for html
set matchpairs+=<:>
set title
set showmatch
set showmode
set ruler
set ignorecase "case insensitive matching 
set smartcase
set incsearch
set hlsearch
" remember that to remove highlights
" temporarily, you can do :nohlsearch

" show me the current filesystem
nnoremap fs :40Lex<CR>
nnoremap tfs :tabe +Ex<CR>


"finding file
"search down into subfolders
"provides tab-completition for file-related taska
set path +=**
set wildmenu

" let's not complicate life with folds
" automatically set fold if a c++ file is opened
"autocmd FileType cpp set foldmethod=syntax
"autocmd FileType hpp set foldmethod=syntax
"" likewise if a python file is opened
"autocmd FileType python set foldmethod=indent
"autocmd FileType python nnoremap <space> za
"autocmd FileType python vnoremap <space> zf


set nocompatible              " be iMproved, required
filetype off                  " required

" set the runtime path to include Vundle and initialize
" ===================================================================================
"set rtp+=~/.vim/bundle/Vundle.vim
"call vundle#begin()
"" alternatively, pass a path where Vundle should install plugins
""call vundle#begin('~/some/path/here')
"
"" let Vundle manage Vundle, required
"Plugin 'VundleVim/Vundle.vim'
"
""" track ultisnips
"Plugin 'SirVer/ultisnips'
"
"" and then add ultisnips
"Plugin 'honza/vim-snippets'
"
""wal colors
"Plugin 'dylanaraps/wal.vim'
"
"" run stuff async (compile and still look at code)
"Plugin 'skywind3000/asyncrun.vim'
"
"Plugin 'ycm-core/YouCompleteMe'
"" The following are examples of different formats supported.
"" Keep Plugin commands between vundle#begin/end.
"" plugin on GitHub repo
"" plugin from http://vim-scripts.org/vim/scripts.html
"" Plugin 'L9'
"" Git plugin not hosted on GitHub
"" git repos on your local machine (i.e. when working on your own plugin)
"" The sparkup vim script is in a subdirectory of this repo called vim.
"" Pass the path to set the runtimepath properly.
"" Install L9 and avoid a Naming conflict if you've already installed a
"" different version somewhere else.
"" Plugin 'ascenator/L9', {'name': 'newL9'}
"
"" All of your Plugins must be added before the following line
"call vundle#end()            " required
"filetype plugin indent on    " required
" ===================================================================================
"
"
" To ignore plugin indent changes, instead use:
"filetype plugin on
"
" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
"
" see :h vundle for more details or wiki for FAQ
" Put your non-Plugin stuff after this line




"adding c completion for ycm
"let g:ycm_clangd_binary_path = "/home/gospodar/.vim/bundle/YouCompleteMe/third_party/ycmd/third_party/clangd/output/bin/clangd"
let g:ycm_clangd_binary_path = "/usr/bin/clangd"
" adding the youcompleteme's global conf for python
let g:ycm_python_interpreter_path = ''
let g:ycm_python_sys_path = []
let g:ycm_extra_conf_vim_data = [
  \  'g:ycm_python_interpreter_path',
  \  'g:ycm_python_sys_path'
  \]
let g:ycm_global_ycm_extra_conf = '~/global_extra_conf.py'

" let me see all of the god damn error messages
let g:ycm_max_diagnostics_to_display=0


" Signature help is triggered in insert mode automatically when g:ycm_auto_trigger is enabled and is not supported when it is not enabled.
" so here we go
" --> it's on by default
"
"  adding keyboard shortcuts for YCMCompleter commands so that i don't need
"  command mode
nmap gsw <Plug>(YCMFindSymbolInWorkspace)
nmap gsd <Plug>(YCMFindSymbolInDocument)
nnoremap gdf :YcmCompleter GoToDefinition<CR>
nnoremap gdc :YcmCompleter GoToDeclaration<CR>
nnoremap gdo :YcmCompleter GoToDocumentOutline<CR>
nnoremap doc :YcmCompleter GetDoc<CR>
nnoremap fix :YcmCompleter FixIt<CR>
nnoremap type :YcmCompleter GetType<CR>
nnoremap diag :YcmShowDetailedDiagnosti<CR>
nnoremap format :YcmCompleter Format<CR>

" and now do the trigger configuration
let g:UltiSnipsExpandTrigger="<c-k>"
let g:UltiSnipsJumpForwardTrigger="<c-k>"
let g:UltiSnipsJumpBackwardTrigger="<c-l>"
let g:UltiSnipsEditSplit="vertical"
let g:UltiSnipsListSnippets="<c-]>"

"colorscheme wal

" configure AsyncRun to be a bit more chill
let g:asyncrun_open = 5
autocmd FileType tex let g:asyncrun_open = 0
nnoremap cl :cclose<CR>


" switched to caps being escape 'couse control is now in a better position
"au VimEnter * !setxkbmap -option caps:ctrl_modifier
"au VimLeave * !setxkbmap -option
"
" my cool commands
" this prints our a python variable 
autocmd FileType python nnoremap <C-p> yiwAprint("pa")€ýaaprint(pa)

" end the W is not a command fiasko
cnoreabbrev <expr> W ((getcmdtype() is# ':' && getcmdline() is# 'W')?('w'):('W'))
