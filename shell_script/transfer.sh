#!/bin/bash

#启动程序使用教程：
#1) 设置此脚本中的<local_path>自定义变量，将路径设置成自己电脑对应的路径
#2）自动化脚本执行->后缀解释:
# trans:将程序压缩包传到机器
# run:启动机器(会新建两个新终端，ssh连接后，自动启动程序，无需其他操作
# transRun:推送程序,并启动机器

# 检查sshpass是否安装
if ! command -V sshpass &>/dev/null; then
  echo "sshpass is not installed. Installing..."
  sudo apt-get update
  sudo apt-get install -y sshpass
  echo "sshpass has been installed successfully."
fi

#自定义变量
local_path="/home/user-e609/workspace/manifest_workspace/rtk_project/robomaker"
#固定变量
remote_host="192.168.10.150"
remote_user="positec"
transfer_port="4321"
remote_password="123"
remote_target_path="/app/data/"
compress_file="ZZZ.tar.gz"
package_path="install_orin"
timeout_seconds=5
#命令变量
ssh_connet_mutual="sdcrun sshpass -p $remote_password ssh -tt $remote_user@$remote_host" #开启伪终端,双向交互
ssh_connet="sdcrun sshpass -p $remote_password ssh -t $remote_user@$remote_host"         #开启伪终端
sudo_su="echo "$remote_password" | sudo -S su"

#--------------------------执行程序------------------------
cd $local_path

#压缩程序
function CompressFile {
  if [ -e "$compress_file" ]; then
    rm "$compress_file"
    echo "File '$compress_file' has been deleted."
  else
    echo "File '$compress_file' dose not exist."
  fi
  echo "=== tar install orin ..."
  tar -cf $compress_file $package_path
  echo "=== local sender md5sum info:"
  md5sum
  $compress_file
}

#远程开启监听,本地发送文件
function DataTransfer {
  #step01->开启新新终端,通过ssh协议控制远程删除原压缩文件,并开启监听
  gnome-terminal -- bash -c "echo start_ssh_connection; sleep 2; \
    #远程设备控制
    $ssh_connect_mutual '
      $sudo_su
      # step01->删除原文件,接收新文件
      echo #空行
      echo !!!!delete_compressed_file!!!
      cd $remote_target_path && rm -rf $compress_file
      echo -----delete_finished------
      echo '=== Listening for incoming data ...'
      nc -l $transfer_port > $compress_file
      # step02->文件接收完成，信息显示
      echo '=== receiver md5sum info:'
      md5sum $compress_file
      sleep 2
    ';
  "
  #step02->本地文件发送
  echo "=== sending data ..."
  sleep 3 #等待ssh远程开启监听(操作远程需要时间)
  sdcrun nc -q1 $remote_host $transfer_port <$compress_file
  echo -e '\033[0;32m===data send over===.\033[0m'
}

#连接机器,启动进程
function StartAll {
  gnome-terminal -- bash -c "echo 'start ssh connection'; sleep 1; \
    # ssh连接,启动进程
    $ssh_connet '
      # 远程控制命令
      echo "$remote_password" | sudo -S su
      echo
      # 启动slam进程
      cd /app/oem
      sudo su -c './start.sh'
      sudo su
      bash -i';
      exec bash
  "
}

#解压文件,并启动程序
function Start {
    # #step01->检查文件传输是否正常
    local remote_md5=$($ssh_connet_mutual "cd $remote_target_path; md5sum $compress_file")
    remote_md5=$(echo "$remote_md5" | awk '{print $1}')
    local local_md5=$(md5sum $compress_file | awk '{print $1}')
    if [ "$local_md5" = "$remote_md5" ]; then
      echo '=== compress file matching ==='
    else
      echo -e '\033[0;31m!!!transfer file not matching!!!\033[0m'
      echo '!!!can not start the machine!!!'
      exit
    fi

  # 远程设备控制
  $ssh_connet_mutual "
    $sudo_su
    cd /app/oem
    rm -rf *
    echo '=== transfer file decompression...'
    tar -xf ${remote_target_path}${compress_file}
    mv $package_path/* .
    sync
  "
  # # ssh连接机器启动程序
  echo '=== start all ...'
  StartAll
  sleep 2 #如果不等待2s,planner线程会崩
  echo '=== start planner ...'
  StartPlanner
}
    
#后缀选择
case $1 in
trans) #transfer
    CompressFile
    DataTransfer
    ;;
run) #run
    StartAll
    sleep 2
    StartPlanner
    ;;
transRun) #transfer and run
    CompressFile
    DataTransfer
    Start
    ;;
*)
;;
esac