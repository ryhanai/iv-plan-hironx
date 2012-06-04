
20120604
vmwareを利用している場合など、グラフィックによっては
vpythonの描画周期が異様に遅くなることがある。
その場合は、Makefile.visual-5.32をMakefileに
リネームして利用するとよい。
ただし、visual.extrusionなど一部の機能が利用できなくなる

現在のところ、ubuntu 10.04, 64bit, nvidia利用の場合は
visual.5.74で正常に動作することが確認されている。
一方で、vmware ubuntu 10.04, 32bit では描画が遅くなる。
