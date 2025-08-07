mark1@cricket:~ $ sudo su　#管理者権限に入る。これをしないと制御ログを作成できず、csvファイルの作成も困難になる。
root@cricket:/home/mark1# cd ..
root@cricket:/home# cd EM
root@cricket:/home/EM# sudo git pull
hint: Pulling without specifying how to reconcile divergent branches is
hint: discouraged. You can squelch this message by running one of the following
hint: commands sometime before your next pull:
hint: 
hint:   git config pull.rebase false  # merge (the default strategy)
hint:   git config pull.rebase true   # rebase
hint:   git config pull.ff only       # fast-forward only
hint: 
hint: You can replace "git config" with "git config --global" to set a default
hint: preference for all repositories. You can also pass --rebase, --no-rebase,
hint: or --ff-only on the command line to override the configured default per
hint: invocation.
remote: Enumerating objects: 24, done.
remote: Counting objects: 100% (24/24), done.
remote: Compressing objects: 100% (22/22), done.
remote: Total 22 (delta 12), reused 0 (delta 0), pack-reused 0 (from 0)
Unpacking objects: 100% (22/22), 13.11 KiB | 167.00 KiB/s, done.
From https://github.com/nhikaru193/EM
   39a267a..51f848c  main       -> origin/main
Updating 39a267a..51f848c
Fast-forward
 C_GOAL_DETECTIVE_ARLISS_rm.py | 764 ------------------------------------------
 GDA2.py                       | 230 +++++++++++++
 test_test.py                  |  48 +--
 3 files changed, 235 insertions(+), 807 deletions(-)
 delete mode 100644 C_GOAL_DETECTIVE_ARLISS_rm.py
 create mode 100644 GDA2.py
root@cricket:/home/EM# sudo systemctl start pigpiod
root@cricket:/home/EM# python3 -u test_test.py | tee /home/EM/_txt/output.txt #この文章で/home/EM/_txt階層に、test_test.pyの制御ログであるoutput.txtファイルを作成する。
これは最初の行です。
これは二番目の行です。
現在時刻: 2025-08-01 13:04:53.221662
root@cricket:/home/EM# 


==============csvファイル書き込み================
current_time_str = time.strftime("%m%d-%H%M%S") #現在時刻をファイル名に含める
filename = f"bme280_data_{current_time_str}.csv"
with open(filename, "w", newline='') as f: # newline='' はCSV書き込みのベストプラクティス #withでファイルを安全に開く＋この実行文を抜けるときに自動でf.close()
    writer = csv.writer(f)
    writer.writerow(["Time", "Pressure(hPa)", "Acceleration_X(m/s^2)", "Acceleration_Y(m/s^2)", "Acceleration_Z(m/s^2)"])
    writer.writerow([e_time, pressure, ax, ay, az])
    f.flush()
    
