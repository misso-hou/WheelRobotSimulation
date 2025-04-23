#!/bin/bash

#数据回放需要自行操作的步骤：
#1）打开仿真开关，编程完成后,会在docker环境中的'home/positec/workspace/csvPlt，路径下生 成csv_plt可执行文件
#也可以直接拷贝别人编译好的回放程序放到指定路径即可
#2）查看宿主机对应的docker环境 "workspace/c§vPlt"路径，将本脚本中的＜12四61^^讣＞变量替 换成本地路径
#3)此脚本中的＜local_ip_address>变量替换成个人电脑ip
#4)此脚本放在与csvPlt路径同级目录（需要启动docker容器，映射路径）
#5)自动化脚本执行-＞后缀解释：
#	pullPlt:拉取设备上的数据并显示（只有这个后缀会执行csv log的拉取的操作，其他的只是做显 示）
#	set:自定义可视化参数, 例如"./csv_log_plot.sh set 10 500",即按照每帧10ms的速度播放，并从500帧后开始播放（可定义4个参数，最后的两个参数暂时用不到，只设置一个参数也可以执行）
#   空:即"./csv_log_plot.sh",绘制csv动画，动画默认以50ms的周期从第一帧开始播放


#自定义变量
local_ip_address="192.168.10.102"	#本器 ip
target_path="/home/user-e609/workspace/manifest_workspace"
#固定变量
remote_host="192.168.10.150" 
remote_user="positec" 
transfer_port="1234" 
remote_password="123" 
remote_csvLog_path="/app/data/log/csvLog" 
csv_file_path="/csvPlt" 
compress_file="csvLog.tar.gz" #docker配置
IMAGE_PATH="dev-docker.positecgroup.com/algorithm/" 
IMAGE_NAME="rockchip-px30-u20.04" 
TAG="alpha-vl.7" 
docker_password="1"
#step0l->进入'csv_plt '文件存储位置（docker与宿主机公有文件夹workspace) 
cd ${target_path}${csv_file_path}

#删除原有压缩包
function DeleteExistFile {
    if [ -e "$compress_file" ]; then
        rm "$compress_file"
        echo "File '$compress_file' has been deleted." 
    else
        echo "File '$compress__file' does not exist." 
    fi
}

#本地开启监听，接收文件，并在后台持续进行
function DataTransAndPLot {
    #打开新终端
    gnome-terminal -- bash -c "
        #本地开启监听，接收文件
        echo '=== Listening for incoming data ...'
        nc -l $transfer_port > $compress_file
        #解压文件
        echo -e '\033[0;32mFile transfer completed successfully.\033[0m'
        tar -xf $compress_file --strip-components=3
        # 进入 docker,调用可视化
        if sudo docker ps --format '{{.Name}}' | grep -Eq '^${IMAGE_NAME}_$ {TAG}$'; then
            echo 'docker container ${IMAGE_NAME}_${TAG} is running' 
            sudo -S docker exec -it ${IMAGE_NAMEj_${TAG} /bin/bash -c ' 
                echo 
                echo enter_docker 
                cd csvPlt 
                ./csv_plt
            '
        else 
            echo 'run ${IMAGE_NAME}_${TAG} docker container'
            sudo docker run -v /tmp/.Xll-unix:/tmp/.Xll-unix \
            -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
            --rm -it --shm-size=lg --name ${IMAGE_NAME}_${TAG} \
            --hostname docker --network host \
            -v $target_path/:/home/positec/workspace ${IMAGE_PATH}${IMAGE_NAME}: ${TAG} /bin/bash -c '
                echo enter_docker 
                cd csvPlt 
                ./csv_plt
            '
        fi
    "

    #step02->通过ssh协议控制发送端设备对csv数据进行压缩并发送
    sdcrun sshpass -p $remote_password ssh -T $remote_user@$remote_host "
        #非交互式设置机器root权限
        echo $remote_password | sudo -S su
        if [ -d "$remote_csvLog_path" ]; then
            echo #换行
            echo 'csv log exists.'
            sudo cp /app/data/map/* $remote_csvLog_path
            sudo cp /app/data/log/*.log $remote_csvLog_path
            sudo cp /app/data/log/*.log.l $remote_csvLog_path
            cd /app/data/log
            sudo tar -czf $compress_file $remote_csvLog_path
            nc -q1 $local_ip_address $transfer_port < $compress file
            echo 'remote device send file'	
            rm $compress_file
        else
            echo #换行
            echo '!!!csvLog does not exist!!!'
        fi
    "
}

#启动docker镜像,并绘图
function RunDocker {
    sudo docker run -v /tmp/.Xll-unix:/tmp/.Xll-unix \
    -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
    --rm -it --shm-size=1g --name ${IMAGE_NAME)_${TAG} \
    --hostname docker --network host \
    -v $target_path/:/home/positec/workspace ${IMAGE_PATH}${IMAGE_NAME}:$ {TAG} /bin/bash -c "
        echo 'enter docker'
        cd csvPlt
        ./csv_plt $1 $2 $3 $4
    "
}

#进入docker容器,绘图
function ExecDocker {
    echo "$docker_password" | sudo -S docker exec -t ${IMAGE_NAME}_${TAG} /bin/bash -c "
        echo 'enter docker'
        cd csvPlt
        ./csv_plt $1 $2 $3 $4
    "
}
        
function DockerPlot {
    #判断docker容器是否正在运行
    if sudo docker ps --format '{{.Names}}' | grep -Eq "^${IMAGE_NAME}_${TAG}$"; then
        echo "docker container ${IMAGE_NAME}_${TAG} is running"
        ExecDocker $@
    else 
        echo "run ${IMAGE_NAME}_${TAG} docker container"
        RunDocker $@
    fi
}

case $1 in
pullPlt)
    #step1->删除原有压缩包
    DeleteExistFile
    #step2->文件收发，收发完成绘图
    DataTransAndPLot
  ;;
set)
    shift
    gnome-terminal -- bash -c "first enter docker; \
        sudo docker exec -it ${IMAGE_NAME}_${TAG} /bin/bash -c '
        cd csvPlt;
        ./csv_plt $1 $2 $3 $4'
        ;
    "
  ;;
*)
    gnome-terminal -- bash -c "first enter docker; \
        sudo docker exec -it ${IMAGE_NAME}_${TAG} /bin/bash -c '
            cd csvPlt;
            ./csv_plt'
        ;
    "
;;
esac


    

