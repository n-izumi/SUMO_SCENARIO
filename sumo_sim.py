from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random  
import time
import datetime
import json
import math
import logging
from concurrent.futures import ThreadPoolExecutor
import threading
import urllib
from urllib import request
import socket
import numpy as np

from flask import Flask, request, jsonify, abort, make_response
from flask_cors import CORS

# Python用に用意されているTraCIのライブラリへのパスを設定する
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

# TraCIの機能を使うためにインポート
import traci
print(os.environ["SUMO_HOME"])

class SumoSim:
    sim_time = 3600
    run_flg = True

    def __init__(self):
        # 引数読み込み
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                            default=False, help="run the commandline version of sumo")
        optParser.add_option("--map", help="map name")
        optParser.add_option("--scenario", help="scenario name")
        optParser.add_option("--scenario-list", help="scenario name list")
        optParser.add_option("--sumo-config", default="sumo.sumocfg", help="sumo config file")
        optParser.add_option("--sim-config", default="Set.json", help="sumo simulation config file")
        optParser.add_option("--seed", type=int, help="random seed")
        optParser.add_option("--time", type=int, help="sim time")
        optParser.add_option("--log", action="store_true",
                            default=False, help="log flag")
        optParser.add_option("--output-directory", default="result", help="output directory")
        optParser.add_option("--state-prefix", action="store_true", dest="statePrefix", default="state",
                            help="prefix for synchronized state files")
        optParser.add_option("--src", action="store_true", help="the remote directory to sync")
        optParser.add_option("--dst", action="store_true", default="states", help="the subdirectory for the synced files")
        optParser.add_option("--delay", default=1,
                            type=float, help="the delay between simulation states")
        optParser.add_option("--iterations", type=int,
                            help="the number of iterations to run (mainly useful for testing)")
        optParser.add_option("-v", "--verbose", action="store_true",
                            default=False, help="tell me what you are doing")
        # remaining command line options are treated as rsync args
        options, args = optParser.parse_args()
        self.set_log_config()

        if not options.scenario and not options.scenario_list:
            print("シナリオを指定してください。")
            sys.exit()

        scenario_name = os.path.join("/home/traffic/SUMO_SCENARIO", options.map, options.scenario)
        self.sim_time = options.time
        
        # 設定ファイル読み込み
        self.settings_name = os.path.join(scenario_name, options.sim_config)
        # print(self.settings_name)
        self.settings = self.setting_read(self.settings_name)
        self.simulation_setting()
        step_length = int(self.settings["FRAME_RATE"]) / 1000

        self.sumo_config = os.path.join(scenario_name, options.sumo_config)

        # ランダムのシード設定
        if options.seed != None:
            random.seed(options.seed)

        # ナンバープレートを格納する変数
        self.vehicle_info = {}

        self.image_size = [1600,1200]
        self.bounding_box_min = [160, 120]
        self.bounding_box_max = [800, 600]

        self.main_node_id = "NODEPC01"
        self.sub_node_id = "NODEPC02"
        # self.branch_node_id = "BoxPC03"
        self.branch_info = self.set_branch_info()
        self.straight_traffic_guide_id = self.settings["STRAIGHT_TRAFFIC_GUIDE"]
        self.regulation_traffic_guide_id = self.settings["REGULATION_TRAFFIC_GUIDE"]
        self.nodes_traffic_guide_id = self.settings["NODES_TRAFFIC_GUIDE"].split(",")
        self.leading_vehicle_info = self.set_leading_vehicle_info()
        self.residence_vehicle_number = {
            "straight": -1,
            "regulation": -1
        }
        self.inside_car_number = -1
        self.congestion_vehicle_number = self.set_congestion_vehicle_number()
        self.go_possible_flag = False
        self.straight_vehicle_number = 0
        self.regulation_vehicle_number = 0

        self.url = self.settings["HTTP_SERVER_URL"]
        # self.sumoCom = sumo_com.SumoCom(self.settings["HTTP_SERVER_URL"], 30, self.settings["HTTP_SERVER_HOST"], self.settings["HTTP_SERVER_PORT"])
        
        self.time_out_flag = False
        self.time_out_history_flag = False
        self.time_out_side = 0
        self.construction_vehicle_time = None
        self.construction_vehicle = 0
        self.send_time_out = 30
        self.host = self.settings["HTTP_SERVER_HOST"]
        self.port = self.settings["HTTP_SERVER_PORT"]
        self.output_directory = options.output_directory
        # self.guide_traffic_light_sim_time = 0
        self.guide_traffic_light = ["-1", "-1"]
        self.collision_caution = False

        if options.nogui:
            sumoBinary = sumolib.checkBinary('sumo')
        else:
            sumoBinary = sumolib.checkBinary('sumo-gui')

        # print(traci.getVersion())
        # this is the normal way of using traci. sumo is started as a
        # # subprocess and then the python script connects and runs
        
        self.api = Flask(__name__)

        self.api.add_url_rule("/", methods=['POST'], view_func=self.post_sim_recv)
        # sumo_args = [sumoBinary, step_length, options.seed]
        # self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="thread")
            
        # with ThreadPoolExecutor(max_workers=2, thread_name_prefix="thread") as executor:
        try:
            thread_sim = threading.Thread(target=self.sumo_run, args=(sumoBinary, step_length, options.seed))
            thread_recv = threading.Thread(target=self.recv, daemon=True)
            thread_sim.start()
            thread_recv.start()
            # self.sumo_thread = self.executor.submit(self.sumo_run, sumoBinary, step_length, options.seed)
            # self.recv_thread = self.executor.submit(self.recv)
        except KeyboardInterrupt:
            print("シミュレーション終了")
            sys.exit()
        # self.run()
            
    def sumo_run(self, sumoBinary, step_length, seed):
        time.sleep(5)
        traci.start(
            [
                sumoBinary,
                "-c", self.sumo_config,
                "--step-length", str(step_length),
                "--delay", "1000",
                "--seed", str(seed),
                "--ignore-junction-blocker", str(60),
                "--time-to-teleport", "-1",
                "--window-size", "1280,1024",
                "--start", "--quit-on-end"
            ]
        )
        self.run()
        traci.close()
        # os.kill(os.getpid(), 9)
        # sys.exit()


    # ログ出力設定
    def set_log_config(self):        
        # SUMO-SIM LOG
        self.sumo_log = logging.getLogger('SUMO')
        formatter = logging.Formatter('%(asctime)s <%(levelname)s> : %(message)s')
        fileHandler = logging.FileHandler('./log/' + str(datetime.datetime.today()) + '-sumo.log', mode='a')
        fileHandler.setFormatter(formatter)
        self.sumo_log.setLevel(logging.INFO)
        self.sumo_log.addHandler(fileHandler)
 
        # SUMO-SERVER LOG
        self.server_log = logging.getLogger('SERVER')
        formatter = logging.Formatter('%(asctime)s <%(levelname)s> : %(message)s')
        fileHandler = logging.FileHandler('./log/server.log', mode='a')
        fileHandler.setFormatter(formatter)
        self.server_log.setLevel(logging.INFO)
        self.server_log.addHandler(fileHandler)

    def run(self):
        """execute the TraCI control loop"""
        step = 0
        count = 0
        mode = "straight"
        tlsFlag = False
        straight_new_tlsState = ""
        straight_old_tlsState = ""
        regulation_new_tlsState = ""
        regulation_old_tlsState = ""
        self.tls_state_list = {}
        self.tls_state_list["straight"] = self.set_traffic_state_signal(self.straight_traffic_guide_id)
        self.tls_state_list["straight"]["id"] = self.straight_traffic_guide_id
        self.tls_state_list["regulation"] = self.set_traffic_state_signal(self.regulation_traffic_guide_id)
        self.tls_state_list["regulation"]["id"] = self.regulation_traffic_guide_id
        if len(self.nodes_traffic_guide_id) > 0 and '' not in self.nodes_traffic_guide_id:
            for i, node_traffic_guide_id in enumerate(self.nodes_traffic_guide_id):
                tls_key = "node" + str(i + 1)
                self.tls_state_list[tls_key] = self.set_traffic_state_signal(node_traffic_guide_id)
                self.tls_state_list[tls_key]["id"] = node_traffic_guide_id
        # self.traffic_guide_change_history()
        # print(self.congestion_vehicle_number)
        # self.set_detector_lane("RO")
        # print(self.tls_state_list)
        # 指定した信号機の状態を設定する
        # traci.trafficlight.setRedYellowGreenState(self.straight_traffic_guide_id, "GG")   # ストレート側
        # traci.trafficlight.setRedYellowGreenState(self.regulation_traffic_guide_id, "GG")   # 規制側
        # dataSet = {'CommandID': "", 'EventID':"", 'TimeStamp':0}     # 送信データの雛形
        # value = {}

        # straightOutLane = traci.lanearea.getLaneID(self.settings["STRAIGHT_SECESSION_DETECTOR"])
        # regulationOutLane = traci.lanearea.getLaneID(self.settings["REGULATION_SECESSION_DETECTOR"])
        # if self.guide_traffic_light == ["1", "1"]:
        #     print("test")
        self.output_header()
        self.sim_start_end_time()

        t_old = -1

        while traci.simulation.getTime() <= self.sim_time:
            traci.simulationStep()
            # 全車両の検出フラグやナンバー割り当て
            self.set_vehicle_info()
            
            # シミュレーション時間の取得
            t_now = traci.simulation.getTime()
            # t_old = datetime.datetime.now()
            # print(self.get_sim_time())

            # 各コマンド処理呼び出し
            
            # 信号機検出
            if self.settings["FLAG_TRAFFIC_LIGHT_RECOGNITION"] == "TRUE":
                # ストレート側信号機
                if self.settings["STRAIGHT_TRAFFIC_LIGHT"] != "":
                    straight_new_tlsState = traci.trafficlight.getPhaseName(self.settings["STRAIGHT_TRAFFIC_LIGHT"])
                    # シミュレータ起動時と信号が切り替わるときに送信
                    if straight_new_tlsState == "" or straight_new_tlsState != straight_old_tlsState:

                        # KPIログ出力
                        self.traffic_light_history(1, straight_new_tlsState)
                        # 信号機認識
                        f001_0000 = self.traffic_light_recognition(self.settings["STRAIGHT_TRAFFIC_LIGHT"], self.main_node_id)
                        straight_old_tlsState = straight_new_tlsState
                        # print(f001_0000)
                        self.sumo_log.info(f001_0000)
                        self.send(self.set_command(f001_0000))
                
                # 規制側信号機
                if self.settings["REGULATION_TRAFFIC_LIGHT"] != "":
                    regulation_new_tlsState = traci.trafficlight.getPhaseName(self.settings["REGULATION_TRAFFIC_LIGHT"])
                    # シミュレータ起動時と信号が切り替わるときに送信
                    if regulation_new_tlsState == "" or regulation_new_tlsState != regulation_old_tlsState:

                        # KPIログ出力
                        self.traffic_light_history(2, regulation_new_tlsState)
                        # 信号機認識
                        f001_0000 = self.traffic_light_recognition(self.settings["REGULATION_TRAFFIC_LIGHT"], self.sub_node_id)
                        regulation_old_tlsState = regulation_new_tlsState
                        # print(f001_0000)
                        self.sumo_log.info(f001_0000)
                        self.send(self.set_command(f001_0000))
                    
            # メインPC(ストレート側処理)
            self.sumo_log.info("-----------------------------メインPC-----------------------------")
            
            # 車列検出（離脱
            if self.settings["FLAG_SECESSION_VEHICLE_DETECTION"] == "TRUE":
                f001_0300 = self.breakaway_vehicle_detection(self.settings["STRAIGHT_SECESSION_DETECTOR"], self.main_node_id)
                # print(f001_0300)
                self.sumo_log.info(f001_0300)
                self.send(self.set_command(f001_0300))

            # 車列検出（接近
            if self.settings["FLAG_APPROACH_VEHICLE_DETECTION"] == "TRUE":
                f001_0400 = self.approaching_vehicle_detection(self.settings["STRAIGHT_APPROACH_DETECTOR"], self.main_node_id)
                # print(f001_0400)
                self.sumo_log.info(f001_0400)
                self.send(self.set_command(f001_0400))

            # ナンバープレート認識
            if self.settings["FLAG_LICENSE_PLATE_RECOGNITION"] == "TRUE":
                f001_0700 = self.license_plate_recognition(self.main_node_id)
                # print(f001_0700)
                self.sumo_log.info(f001_0700)
                self.send(self.set_command(f001_0700))

            # 車両認識（渋滞カメラ
            if self.settings["FLAG_VEHICLE_RECOGNITION_TJ"] == "TRUE":
                f001_0800 = self.vehicle_recognition_TJ(self.main_node_id)
                # print(f001_0800)
                self.sumo_log.info(f001_0800)
                self.send(self.set_command(f001_0800))

            # 車両認識（ナンバープレート）
            if self.settings["FLAG_VEHICLE_RECOGNITION_NP"] == "TRUE":
                f001_0900 = self.vehicle_recognition_NP(self.main_node_id)
                # print(f001_0900)
                self.sumo_log.info(f001_0900)
                self.send(self.set_command(f001_0900))

            # 車速・距離検出
            if self.settings["FLAG_SPEED_DISTANCE_RECOGNITION"] == "TRUE":
                f002_0000 = self.speed_and_distance_recognition(self.settings["STRAIGHT_APPROACH_DETECTOR"], self.main_node_id)
                # print(f002_0000)
                self.sumo_log.info(f002_0000)
                self.send(self.set_command(f002_0000))
                #speed_and_distance_recognition(self.settings["STRAIGHT_SECESSION_DETECTOR"])

            # サブPC(規制側処理)
            self.sumo_log.info("-----------------------------サブPC-----------------------------")

            # 車列検出（離脱
            if self.settings["FLAG_SECESSION_VEHICLE_DETECTION"] == "TRUE":
                f001_0300 = self.breakaway_vehicle_detection(self.settings["REGULATION_SECESSION_DETECTOR"], self.sub_node_id)
                # print(f001_0300)
                self.sumo_log.info(f001_0300)
                self.send(self.set_command(f001_0300))

            # 車列検出（接近
            if self.settings["FLAG_APPROACH_VEHICLE_DETECTION"] == "TRUE":
                f001_0400 = self.approaching_vehicle_detection(self.settings["REGULATION_APPROACH_DETECTOR"], self.sub_node_id)
                # print(f001_0400)
                self.sumo_log.info(f001_0400)
                self.send(self.set_command(f001_0400))

            # ナンバープレート認識
            if self.settings["FLAG_LICENSE_PLATE_RECOGNITION"] == "TRUE":
                f001_0700 = self.license_plate_recognition(self.sub_node_id)
                # print(f001_0700)
                self.sumo_log.info(f001_0700)
                self.send(self.set_command(f001_0700))

            # 車両認識（渋滞カメラ
            if self.settings["FLAG_VEHICLE_RECOGNITION_TJ"] == "TRUE":
                f001_0800 = self.vehicle_recognition_TJ(self.sub_node_id)
                # print(f001_0800)
                self.sumo_log.info(f001_0800)
                self.send(self.set_command(f001_0800))

            # 車両認識（ナンバープレート）
            if self.settings["FLAG_VEHICLE_RECOGNITION_NP"] == "TRUE":
                f001_0900 = self.vehicle_recognition_NP(self.sub_node_id)
                # print(f001_0900)
                self.sumo_log.info(f001_0900)
                self.send(self.set_command(f001_0900))

            # 車速・距離検出
            if self.settings["FLAG_SPEED_DISTANCE_RECOGNITION"] == "TRUE":
                f002_0000 = self.speed_and_distance_recognition(self.settings["REGULATION_APPROACH_DETECTOR"], self.sub_node_id)
                # print(f002_0000)
                self.sumo_log.info(f002_0000)
                self.send(self.set_command(f002_0000))
                #speed_and_distance_recognition(self.settings["STRAIGHT_SECESSION_DETECTOR"])

            # 枝道処理
            self.sumo_log.info("-----------------------------枝道-----------------------------")
            
            # 枝道の数分ループ
            for branch_node in self.branch_info:
                node_id = branch_node["node_id"]
                # print(node_id)
                # ナンバープレート認識
                if self.settings["FLAG_LICENSE_PLATE_RECOGNITION"] == "TRUE":
                    f001_0700 = self.license_plate_recognition(node_id)
                    # print(f001_0700)
                    self.sumo_log.info(f001_0700)
                    self.send(self.set_command(f001_0700))

                # 車両認識（渋滞カメラ
                if self.settings["FLAG_VEHICLE_RECOGNITION_TJ"] == "TRUE":
                    f001_0800 = self.vehicle_recognition_TJ(node_id)
                    # print(f001_0800)
                    self.sumo_log.info(f001_0800)
                    self.send(self.set_command(f001_0800))

                # 車両認識（ナンバープレート）
                if self.settings["FLAG_VEHICLE_RECOGNITION_NP"] == "TRUE":
                    f001_0900 = self.vehicle_recognition_NP(node_id)
                    # print(f001_0900)
                    self.sumo_log.info(f001_0900)
                    self.send(self.set_command(f001_0900))


            # 工事帯進入・離脱
            self.sumo_log.info("-----------------------------工事帯進入・離脱検出-----------------------------")
            if self.settings["FLAG_PENETRATION_BREAKAWAY"] == "TRUE":
                # if traci.inductionloop.getLastStepVehicleNumber("straightIn"):
                f006_0000 = self.penetration_breakaway(self.main_node_id)
                # print(f006_0000)   # 工事帯進入・離脱検出
                if f006_0000:
                    self.sumo_log.info(f006_0000)   # 工事帯進入・離脱検出
                    self.send(self.set_command(f006_0000))
                f006_0000 = self.penetration_breakaway(self.sub_node_id)
                if f006_0000:
                    # print(f006_0000)   # 工事帯進入・離脱検出
                    self.sumo_log.info(f006_0000)   # 工事帯進入・離脱検出
                    self.send(self.set_command(f006_0000))


            # １秒ごとに送信するロジック
            if step > 0:
                t_delta = t_now - t_old
                if t_delta >= 1:
                    if self.settings["FLAG_VEHICLE_DETECTION_FRONT_SENSOR"] == "TRUE":
                        self.sumo_log.info("-----------------------------工事帯センサ前車両検出-----------------------------")
                        f006_0100 = self.vehicle_detection_front_sensor(self.main_node_id)
                        # print(f006_0100)   # 工事帯センサ前車両検出
                        self.sumo_log.info(f006_0100)   # 工事帯センサ前車両検出
                        self.send(self.set_command(f006_0100))
                        f006_0100 = self.vehicle_detection_front_sensor(self.sub_node_id)
                        # print(f006_0100)   # 工事帯センサ前車両検出
                        self.sumo_log.info(f006_0100)   # 工事帯センサ前車両検出
                        self.send(self.set_command(f006_0100))
                    t_old = t_now

            # タイムアウト処理
            self.time_out()

            # タイムアウト発生履歴
            # print("----------タイムアウト発生履歴----------")
            self.time_out_history()
            # print("----------先頭車両接近・停止履歴----------")
            self.output_leading_vehicle_state_history()
            # print("----------滞留発生履歴----------")
            self.residence_occurred_history()
            # print("----------工事帯内車両台数履歴----------")
            self.inside_car_number_history()
            # print("----------渋滞発生履歴----------")
            self.congestion_history()
            # print("----------すすめ可能からの待ち時間履歴----------")
            self.waiting_time_history()
            # print("----------衝突検知----------")
            self.collision_judge()

            # マップ内から消えた車両IDの情報をnumber_dictから消去
            allVehicleIDs = traci.vehicle.getIDList()
            for vehicleId in self.vehicle_info:
                if vehicleId not in allVehicleIDs:
                    self.vehicle_info.pop(str(vehicleId)) 
                break

            step += 1
            count +=1

        self.sumo_log.info("---straightNumber: " + str(self.straight_vehicle_number))
        self.sumo_log.info("---regulationNumber: " + str(self.regulation_vehicle_number))
        print("---straightNumber: " + str(self.straight_vehicle_number))
        print("---regulationNumber: " + str(self.regulation_vehicle_number))
        self.sim_start_end_time()
        return


    def get_options(self):
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                            default=False, help="run the commandline version of sumo")
        options, args = optParser.parse_args()
        return options

    def get_time(self):
        now = datetime.datetime.now()
        timeStamp = (int(time.mktime(now.timetuple()) * 1000)) + int(now.microsecond / 1000)
        return timeStamp

    def get_sim_time(self):
        sim_time = traci.simulation.getCurrentTime()
        sim_time = str(sim_time).zfill(9)
        return sim_time
        
    # 衝突検知
    def collision_judge(self):
        s_veh_ids = traci.lanearea.getLastStepVehicleIDs(self.settings["STRAIGHT_CONSTRUCTION_BAND_DETECTOR"])
        r_veh_ids = traci.lanearea.getLastStepVehicleIDs(self.settings["REGULATION_CONSTRUCTION_BAND_DETECTOR"])

        # ストレート側と規制側のどちらかでも車両なしの場合、衝突なしと判断
        if len(s_veh_ids) == 0 or len(r_veh_ids) == 0:
            return False
            
        #ストレート車線の車両位置情報
        for s_veh_id in s_veh_ids:
            #print(sPosition)
            #print(sPosition[1])
            # 規制車線の車両位置情報
            for r_veh_id in r_veh_ids:
                if self.collision_judge_calc(s_veh_id, r_veh_id):
                    return True
            #print(rPosition)

    # 
    def collision_judge_calc(self, s_vehicle_id, r_vehicle_id):
        sPosition = traci.vehicle.getPosition(s_vehicle_id)
        rPosition = traci.vehicle.getPosition(r_vehicle_id)
        s = np.array(list(sPosition))
        r = np.array(list(rPosition))
        distance = np.linalg.norm(s-r)
        if abs(distance) < 2.0:
            print("衝突を検知しました")
            self.sumo_log.info("-----衝突を検知しました-----")
            self.collision_history_output(s_vehicle_id, r_vehicle_id)
            return True

        return False

    # 信号機認識
    def traffic_light_recognition(self, tlsID, nodeID):
        command = {}
        valuelist = []
        value = {}
        #print(traci.trafficlight.getAllProgramLogics(tlsID))
        programId = traci.trafficlight.getProgram(tlsID)
        timeStamp = self.get_time()
        state = traci.trafficlight.getPhaseName(tlsID)
        greenTime = 0
        yellowTime = 0
        redTime = 0
        for logic in traci.trafficlight.getAllProgramLogics(tlsID):
            # print(logic)
            # print(programId)
            if logic.programID != programId:
                continue

            for phase in logic.phases:
                if phase.name == "Green":
                    greenTime += phase.duration
                elif phase.name == "Yellow":
                    yellowTime += phase.duration
                if phase.name == "Red":
                    redTime += phase.duration
            # greenTime = logic.phases[0].duration
            # yellowTime = logic.phases[1].duration
            # redTime = logic.phases[2].duration
            # redTime += logic.phases[3].duration
            # print(greenTime, yellowTime, redTime)
            break
        if state == "Red":
            color = 2
        elif state == "Yellow":
            color = 1
        elif state == "Green":
            color = 0
        else:
            color = -1

        command['CommandID'] = "0xF0010000"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['Color'] = str(color)
        value['GreenTime'] = str(int(greenTime * 1000))
        value['RedTime'] = str(int(redTime * 1000))
        value['YellowTime'] = str(int(yellowTime * 1000))
        value['ChangeTimeStamp'] = str(self.get_time())
        value['TopLeft'] = ""
        value['BottomRight'] = ""
        value['ImageSize'] = self.set_image_size()
        value['Score'] = str(1.0)
        valuelist.append(value)
        command['Value'] = valuelist

        return command

    # 離脱車両検出
    def breakaway_vehicle_detection(self, detectorID, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        value = {}
        timeStamp = self.get_time()
        vehicle_count = 0
        traffic_jam_length = 0
        vehicleTime = []
        #print(vehicleIDs)

        for detector_id in detectorID.split(","):
            vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detector_id)
            if len(vehicleIDs) != None:
                for vehicle in vehicleIDs:
                    # 検出率計算し、誤検出の場合次の車両へ
                    if not self.vehicle_info[str(vehicle)]["CarsDetectionFlag"]:
                        continue

                    # 車両位置取得
                    vehicle_position = self.set_vehicle_position(False, detector_id, vehicle, True)

                    # 車両全体が検出器に入っていない場合次に車両へ
                    if vehicle_position == None:
                        continue

                    # 車両が検出範囲内に入っていない場合次に車両へ
                    if vehicle_position < float(self.settings["TRAFFIC_JAM_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["TRAFFIC_JAM_DETECTION_DISTANCE_MAX"]):
                        continue

                    vehicle_count += 1
                    stopTime = traci.vehicle.getWaitingTime(vehicle)
                    if traffic_jam_length < traci.lanearea.getJamLengthMeters(detector_id):
                        traffic_jam_length = traci.lanearea.getJamLengthMeters(detector_id)
                    vehicleTime.append(str(stopTime))
            
        vehicleTime_str = ",".join(vehicleTime)

        command['CommandID'] = "0xF0010300"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['VehicleCount'] = str(vehicle_count)
        value['LineLength'] = str(traffic_jam_length)
        value['VehicleTime'] = vehicleTime_str
        value['TopLeft'] = ""
        value['BottomRight'] = ""
        value['ImageSize'] = ""
        value['Score'] = ""
        valuelist.append(value)
        command['Value'] = valuelist

        #print(command)
        return command

    # 接近車両検出
    def approaching_vehicle_detection(self, detectorID, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        value = {}
        timeStamp = self.get_time()
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detectorID)
        vehicle_count = 0
        vehicleTime = []
        
        if len(vehicleIDs) != None:
            for vehicle in vehicleIDs:
                # 検出率計算し、誤検出の場合次の車両へ
                if not self.vehicle_info[str(vehicle)]["CarsDetectionFlag"]:
                    continue

                # 車両位置取得
                vehicle_position = self.set_vehicle_position(True, detectorID, vehicle, True)

                # 車両全体が検出器に入っていない場合次に車両へ
                if vehicle_position == None:
                    continue

                # 車両が検出範囲内に入っていない場合次に車両へ
                if vehicle_position < float(self.settings["TRAFFIC_JAM_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["TRAFFIC_JAM_DETECTION_DISTANCE_MAX"]):
                    continue

                vehicle_count += 1
                stopTime = traci.vehicle.getWaitingTime(vehicle)
                vehicleTime.append(str(stopTime))
                #print(stopTime)
            
        vehicleTime_str = ",".join(vehicleTime)

        command['CommandID'] = "0xF0010400"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['VehicleCount'] = str(vehicle_count)
        value['LineLength'] = str(traci.lanearea.getJamLengthMeters(detectorID))
        value['VehicleTime'] = vehicleTime_str
        value['TopLeft'] = ""
        value['BottomRight'] = ""
        value['ImageSize'] = ""
        value['Score'] = ""
        valuelist.append(value)
        command['Value'] = valuelist

        #print(command)
        return command

    # 車両認識（停止線-工事帯間枝道）
    def vehicle_recognition_stopline_byload(self, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        timeStamp = self.get_time()
        # tls_id = ""
        if nodeID == self.main_node_id:
            approach_detector_id = self.settings["NODE_STRAIGHT_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
            # tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        elif nodeID == self.sub_node_id:
            approach_detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")
            # tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]


        command['CommandID'] = "0xF0010600"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        # 接近車両の情報セット
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(approach_detector_id)
        laneareaLength = traci.lanearea.getLength(approach_detector_id)
        for id in vehicleIDs:
            vehicleState = 0
            value = {}
            # 検出率計算し、誤検出の場合次の車両へ
            if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                continue

            # # 
            # if not self.detection_lane_judge(approach_detector_id, tls_id, id, True):
            #     continue
            
            # 車両位置取得
            vehicle_position = self.set_vehicle_position(True, approach_detector_id, id, True)

            # 車両全体が検出器に入っていない場合次に車両へ
            if vehicle_position == None:
                continue

            # バウンディングボックス処理
            vehicleBoundingBox = self.bounding_box(True, laneareaLength, vehicle_position)[0]
            value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
            value['VehicleState'] = str(vehicleState)
            value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
            value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
            value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
            value['ImageSize'] = self.set_image_size()
            value['Score'] = "100"
            valuelist.append(value)
        
        # 離脱車両の情報セット
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(secession_detector_id)
        laneareaLength = traci.lanearea.getLength(secession_detector_id)
        for id in reversed(vehicleIDs):
            vehicleState = 1
            value = {}
            # 検出率計算し、誤検出の場合次の車両へ
            if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                continue

            # 
            if not self.detection_lane_judge(secession_detector_id, tls_id, id, False):
                continue
            
            # 車両位置取得
            vehicle_position = self.set_vehicle_position(False, secession_detector_id, id, True)

            # 車両全体が検出器に入っていない場合次に車両へ
            if vehicle_position == None:
                continue

            # バウンディングボックス処理
            vehicleBoundingBox = self.bounding_box(False, laneareaLength, vehicle_position)[0]

            value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
            value['VehicleState'] = str(vehicleState)
            value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
            value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
            value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
            value['ImageSize'] = self.set_image_size()
            value['Score'] = "100"
            valuelist.append(value)
        
        command['Value'] = valuelist

        # print("Command: " + str(command))
        return command


    # ナンバープレート認識
    def license_plate_recognition(self, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        timeStamp = self.get_time()
        approach_detector_lane = None
        secession_detector_lane = None
        tls_id = ""
        if nodeID == self.main_node_id:
            approach_detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        elif nodeID == self.sub_node_id:
            approach_detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]
        else:
            branch_data = self.get_branch_data(nodeID)
            # print(branch_data)
            approach_detector_id = branch_data["approach"]["id"][0]
            secession_detector_id = branch_data["secession"]["id"]


        command['CommandID'] = "0xF0010700"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        # 接近車両
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(approach_detector_id)
        laneareaLength = traci.lanearea.getLength(approach_detector_id)
        pos_score = "100"
        number_score = "100"
        for i, id in enumerate(vehicleIDs):

            # 車両認識で誤検出の場合次の車両へ
            if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                continue

            # 
            if not self.detection_lane_judge(approach_detector_id, tls_id, id, True):
                continue
                
            # 車両位置取得
            vehicle_position = self.set_vehicle_position(True, approach_detector_id, id, True)

            # 車両全体が検出器に入っていない場合次に車両へ
            if vehicle_position == None:
                continue

            # 車両が検出範囲内に入っていない場合次に車両へ
            if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                continue

            # バウンディングボックス処理
            vehicleBoundingBox = self.bounding_box(True, laneareaLength, vehicle_position)[1]

            vehicleState = 0
            stopTime = str(traci.vehicle.getWaitingTime(id))
            vehicleType = self.vehicle_info[str(id)]["VehicleType"]
            number = self.vehicle_info[str(id)]["OutputNumber"]
            topLeft = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
            bottomRight = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
            imageSize = self.set_image_size()
            # 取得対象が先頭車両のみかどうか
            if self.settings["FLAG_NUMBER_TARGET_FIRST_VEHICLE"] == "TRUE" and i > 0:
                stopTime = ""
                vehicleType = "-9"
                number = ""
                topLeft = ""
                bottomRight = ""
                imageSize = ""
                pos_score = ""
                number_score = ""
            value = {}
            value['Number'] = number
            value['Hiragana'] = ""
            value['AreaCode'] = ""
            value['Kanji'] = ""
            value['Color'] = "-1"
            value['StopTime'] = stopTime
            value['VehicleType'] = vehicleType
            value['VehicleState'] = str(vehicleState)
            value['TopLeft'] = topLeft
            value['BottomRight'] = bottomRight
            value['ImageSize'] = imageSize
            value['PosScore'] = pos_score
            value['NumberScore'] = number_score
            valuelist.append(value)
            #print(valuelist)
            
        # 離脱車両
        # print(secession_detector_id)
        for detector_id in secession_detector_id:
            vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detector_id)
            laneareaLength = traci.lanearea.getLength(detector_id)
            pos_score = "100"
            number_score = "100"
            for i, id in enumerate(reversed(vehicleIDs)):
                # print(vehicleIDs)

                # 車両認識で誤検出の場合次の車両へ
                if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                    continue

                # 
                if not self.detection_lane_judge(detector_id, tls_id, id, True):
                    continue

                # 車両位置取得
                vehicle_position = self.set_vehicle_position(False, detector_id, id, True)

                # 車両全体が検出器に入っていない場合次に車両へ
                if vehicle_position == None:
                    continue

                # 車両が検出範囲内に入っていない場合次に車両へ
                if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                    continue

                # バウンディングボックス処理
                vehicleBoundingBox = self.bounding_box(False, laneareaLength, vehicle_position)[1]

                vehicleState = 1
                stopTime = str(traci.vehicle.getWaitingTime(id))
                vehicleType = self.vehicle_info[str(id)]["VehicleType"]
                number = self.vehicle_info[str(id)]["OutputNumber"]
                topLeft = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
                bottomRight = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
                imageSize = self.set_image_size()
                # print(vehicle_position)
                # print(i)
                # print(number)
                # 取得対象が先頭車両のみかどうか
                if self.settings["FLAG_NUMBER_TARGET_FIRST_VEHICLE"] == "TRUE" and i > 0:
                    stopTime = ""
                    vehicleType = "-9"
                    number = ""
                    topLeft = ""
                    bottomRight = ""
                    imageSize = ""
                    pos_score = ""
                    number_score = ""
                value = {}
                value['Number'] = number
                value['Hiragana'] = ""
                value['AreaCode'] = ""
                value['Kanji'] = ""
                value['Color'] = "-1"
                value['StopTime'] = stopTime
                value['VehicleType'] = vehicleType
                value['VehicleState'] = str(vehicleState)
                value['TopLeft'] = topLeft
                value['BottomRight'] = bottomRight
                value['ImageSize'] = imageSize
                value['PosScore'] = pos_score
                value['NumberScore'] = number_score
                valuelist.append(value)
                #print(valuelist)
        
        # print(approach_detector_id)
        # print(secession_detector_id)

        # 車両未検出の場合
        if len(valuelist) == 0:
            value = {}
            value['Number'] = ""
            value['Hiragana'] = ""
            value['AreaCode'] = ""
            value['Kanji'] = ""
            value['Color'] = "-1"
            value['StopTime'] = ""
            value['VehicleType'] = "-1"
            value['VehicleState'] = "-1"
            value['TopLeft'] = ""
            value['BottomRight'] = ""
            value['ImageSize'] = ""
            value['PosScore'] = ""
            value['NumberScore'] = ""
            valuelist.append(value)
        
        command['Value'] = valuelist

        # print(command)
        return command

    # 車両認識（渋滞カメラ）
    def vehicle_recognition_TJ(self, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        timeStamp = self.get_time()
        approach_detector_lane = None
        secession_detector_lane = None
        tls_id = ""
        if nodeID == self.main_node_id:
            approach_detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        elif nodeID == self.sub_node_id:
            approach_detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]
        else:
            branch_data = self.get_branch_data(nodeID)
            approach_detector_id = branch_data["approach"]["id"][0]
            secession_detector_id = branch_data["secession"]["id"]


        command['CommandID'] = "0xF0010800"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        # 接近車両の情報セット
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(approach_detector_id)
        # print(traci.lanearea.getLaneIDs(approach_detector_id))
        laneareaLength = traci.lanearea.getLength(approach_detector_id)
        for id in vehicleIDs:
            vehicleState = 0
            value = {}
            # 検出率計算し、誤検出の場合次の車両へ
            if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                continue

            # 
            if not self.detection_lane_judge(approach_detector_id, tls_id, id, True):
                continue
            
            # 車両位置取得
            vehicle_position = self.set_vehicle_position(True, approach_detector_id, id, True)

            # 車両全体が検出器に入っていない場合次に車両へ
            if vehicle_position == None:
                continue

            # 車両が検出範囲内に入っていない場合次に車両へ
            if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                continue

            # バウンディングボックス処理
            vehicleBoundingBox = self.bounding_box(True, laneareaLength, vehicle_position)[0]

            value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
            value['VehicleState'] = str(vehicleState)
            value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
            value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
            value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
            value['ImageSize'] = self.set_image_size()
            value['Score'] = "100"
            valuelist.append(value)
        
        # 離脱車両の情報セット
        for detector_id in secession_detector_id:
            vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detector_id)
            laneareaLength = traci.lanearea.getLength(detector_id)
            for id in reversed(vehicleIDs):
                vehicleState = 1
                value = {}
                # 検出率計算し、誤検出の場合次の車両へ
                if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                    continue

                # 
                if not self.detection_lane_judge(detector_id, tls_id, id, True):
                    continue
                
                # 車両位置取得
                vehicle_position = self.set_vehicle_position(False, detector_id, id, True)

                # 車両全体が検出器に入っていない場合次に車両へ
                if vehicle_position == None:
                    continue

                # 車両が検出範囲内に入っていない場合次に車両へ
                if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                    continue

                # バウンディングボックス処理
                vehicleBoundingBox = self.bounding_box(False, laneareaLength, vehicle_position)[0]

                value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
                value['VehicleState'] = str(vehicleState)
                value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
                value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
                value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
                value['ImageSize'] = self.set_image_size()
                value['Score'] = "100"
                valuelist.append(value)

        # 車両未検出の場合
        if len(valuelist) == 0:
            value = {}
            value['VehicleType'] = "-1"
            value['VehicleState'] = "-1"
            value['StopTime'] = ""
            value['TopLeft'] = ""
            value['BottomRight'] = ""
            value['ImageSize'] = ""
            value['Score'] = ""
            valuelist.append(value)
        
        command['Value'] = valuelist

        # print("Command: " + str(command))
        return command

    # 車両認識（ナンバープレート）
    def vehicle_recognition_NP(self, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        timeStamp = self.get_time()
        inFlag = True
        tls_id = ""
        if nodeID == self.main_node_id:
            approach_detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        elif nodeID == self.sub_node_id:
            approach_detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")
            tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]
        else:
            branch_data = self.get_branch_data(nodeID)
            approach_detector_id = branch_data["approach"]["id"][0]
            secession_detector_id = branch_data["secession"]["id"]

        command['CommandID'] = "0xF0010900"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        # 接近車両の情報セット
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(approach_detector_id)
        laneareaLength = traci.lanearea.getLength(approach_detector_id)
        for id in vehicleIDs:
            vehicleState = 0
            value = {}
            # 検出率計算し、誤検出の場合次の車両へ
            if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                continue

            # 
            if not self.detection_lane_judge(approach_detector_id, tls_id, id, True):
                continue
            
            # 車両位置取得
            vehicle_position = self.set_vehicle_position(True, approach_detector_id, id, True)

            # 車両全体が検出器に入っていない場合次に車両へ
            if vehicle_position == None:
                continue

            # 車両が検出範囲内に入っていない場合次に車両へ
            if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                continue

            # バウンディングボックス処理
            vehicleBoundingBox = self.bounding_box(True, laneareaLength, vehicle_position)[0]

            value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
            value['VehicleState'] = str(vehicleState)
            value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
            value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
            value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
            value['ImageSize'] = self.set_image_size()
            value['Score'] = "100"
            valuelist.append(value)
        
        # 離脱車両の情報セット
        for detector_id in secession_detector_id:
            vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detector_id)
            laneareaLength = traci.lanearea.getLength(detector_id)
            for id in reversed(vehicleIDs):
                vehicleState = 1
                value = {}
                # 検出率計算し、誤検出の場合次の車両へ
                if not self.vehicle_info[str(id)]["VehicleRecognitionFlag"]:
                    continue

                # 
                if not self.detection_lane_judge(detector_id, tls_id, id, True):
                    continue

                # 車両位置取得
                vehicle_position = self.set_vehicle_position(False, detector_id, id, True)

                # 車両全体が検出器に入っていない場合次に車両へ
                if vehicle_position == None:
                    continue

                # 車両が検出範囲内に入っていない場合次に車両へ
                if vehicle_position < float(self.settings["VEHICLE_DETECTION_DISTANCE_MIN"]) or vehicle_position > float(self.settings["VEHICLE_DETECTION_DISTANCE_MAX"]):
                    continue

                # バウンディングボックス処理
                vehicleBoundingBox = self.bounding_box(False, laneareaLength, vehicle_position)[0]

                value['VehicleType'] = self.vehicle_info[str(id)]["VehicleType"]
                value['VehicleState'] = str(vehicleState)
                value['StopTime'] = str(traci.vehicle.getWaitingTime(id))
                value['TopLeft'] = ",".join([str(vehicleBoundingBox[0]), str(vehicleBoundingBox[1])])
                value['BottomRight'] = ",".join([str(vehicleBoundingBox[2]), str(vehicleBoundingBox[3])])
                value['ImageSize'] = self.set_image_size()
                value['Score'] = "100"
                valuelist.append(value)
        
        # 車両未検出の場合
        if len(valuelist) == 0:
            value = {}
            value['VehicleType'] = "-1"
            value['VehicleState'] = "-1"
            value['StopTime'] = ""
            value['TopLeft'] = ""
            value['BottomRight'] = ""
            value['ImageSize'] = ""
            value['Score'] = ""
            valuelist.append(value)

        command['Value'] = valuelist

        # print("Command: " + str(command))
        return command

    # 車速・距離検出
    def speed_and_distance_recognition(self, detectorID, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []  
        timeStamp = self.get_time()
        if nodeID == self.main_node_id:
            approach_detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
        elif nodeID == self.sub_node_id:
            approach_detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
            secession_detector_id = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")

        command['CommandID'] = "0xF0020000"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        # 接近車両
        vehicleIDs = traci.lanearea.getLastStepVehicleIDs(approach_detector_id)
        for id in vehicleIDs:
            # 検出率計算し、誤検出の場合次の車両へ
            speed = (traci.vehicle.getSpeed(id) * 3600 / 1000)
            speed = math.floor(speed * 10 ** 1) / (10 ** 1)
            if not self.vehicle_info[str(id)]["SpeedDistanceFlag"] or speed == 0.0:
                continue
            value = {}
            #print(data)
            distance = self.set_vehicle_position(True, approach_detector_id, id, False)
            if distance == None:
                continue
                
            # 車両が検出範囲内に入っていない場合次に車両へ
            if distance < float(self.settings["SENSOR_DETECTION_DISTANCE_MIN"]) or distance > float(self.settings["SENSOR_DETECTION_DISTANCE_MAX"]):
                continue

            distance = math.floor(distance * 10 ** 1) / (10 ** 1)
            #print(data)
            value['ComFlag'] = "1"
            value['Distance'] = str(distance)
            value['Speed'] = str(speed)
            if len(valuelist) < 6:
                valuelist.append(value)

        # 離脱車両
        for detector_id in secession_detector_id:
            vehicleIDs = traci.lanearea.getLastStepVehicleIDs(detector_id)
            laneareaLength = traci.lanearea.getLength(detector_id)
        # vehicleIDs = traci.lanearea.getLastStepVehicleIDs(secession_detector_id)
            for id in reversed(vehicleIDs):
                speed = (traci.vehicle.getSpeed(id) * 3600 / 1000)
                speed = math.floor(speed * 10 ** 1) / (10 ** 1)
                if not self.vehicle_info[str(id)]["SpeedDistanceFlag"] or speed == 0.0:
                    continue
                value = {}
                #print(data)
                distance = self.set_vehicle_position(False, detector_id, id, False)
                if distance == None:
                    continue

                # 車両が検出範囲内に入っていない場合次に車両へ
                if distance < float(self.settings["SENSOR_DETECTION_DISTANCE_MIN"]) or distance > float(self.settings["SENSOR_DETECTION_DISTANCE_MAX"]):
                    continue

                distance = math.floor(distance * 10 ** 1) / (10 ** 1)
                #print(data)
                value['ComFlag'] = "0"
                value['Distance'] = str(distance)
                value['Speed'] = str(speed)
                if len(valuelist) < 6:
                    valuelist.append(value)

        # 車両台数が5台以下の場合
        while len(valuelist) < 6:
            value = {}
            value['ComFlag'] = "-1"
            value['Distance'] = "0.0"
            value['Speed'] = "0.0"
            valuelist.append(value)
        
        # ソート
        valuelist = sorted(valuelist, key=lambda x: float(x["Speed"]), reverse=True)
        valuelist = sorted(valuelist, key=lambda x: float(x["Distance"]), reverse=False)
        valuelist = sorted(valuelist, key=lambda x: int(x["ComFlag"]), reverse=True)

        command['Value'] = valuelist

        return command

    # 工事帯進入/離脱検出
    def penetration_breakaway(self, nodeID):
        command = {}     # 送信データの雛形
        valuelist = []
        value = {}
        timeStamp = self.get_time()
        sendFlag = False
        direction = -1
        lane_kind = -1
        if nodeID == self.main_node_id:
            approach_detector_id = "straightIn"
            secession_detector_id = "regulationOut"
            lane_kind = 1
        elif nodeID == self.sub_node_id:
            approach_detector_id = "regulationIn"
            secession_detector_id = "straightOut"
            lane_kind = 2

        # 接近車両
        vehicleIDs = traci.inductionloop.getLastStepVehicleIDs(approach_detector_id)
        for id in vehicleIDs:
            if str(id) not in self.vehicle_info.keys():
                continue
            if self.vehicle_info[str(id)]["EntryFlag"] == "False":
                self.vehicle_info[str(id)]["EntryFlag"] = "True"
                sendFlag = True
                if nodeID == self.main_node_id:
                    self.straight_vehicle_number += 1
                if nodeID == self.sub_node_id:
                    self.regulation_vehicle_number += 1
                # direction = 0

        # 離脱車両
        vehicleIDs = traci.inductionloop.getLastStepVehicleIDs(secession_detector_id)
        for id in vehicleIDs:
            if str(id) not in self.vehicle_info.keys():
                continue
            if self.vehicle_info[str(id)]["BreakawayFlag"] == "False":
                self.vehicle_info[str(id)]["BreakawayFlag"] = "True"
                sendFlag = True
                self.passing_vehicle_history(lane_kind, id)
                # direction = 1

        if not sendFlag:
            return None

        command['CommandID'] = "0xF0060000"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['Direction'] = str(direction)
        command['Value'] = value

        return command

    # 工事帯センサ前車両検出
    def vehicle_detection_front_sensor(self, nodeID):
        command = {}     # 送信データの雛形
        value = {}
        # valuelist = []
        timeStamp = self.get_time()

        if nodeID == self.main_node_id:
            approach_detector_id = "straightIn"
            secession_detector_id = "straightOut"
        elif nodeID == self.sub_node_id:
            approach_detector_id = "regulationIn"
            secession_detector_id = "regulationOut"

        # if traci.inductionloop.getLastStepVehicleNumber(approach_detector_id) > 0 or traci.inductionloop.getLastStepVehicleNumber(secession_detector_id):
        if traci.inductionloop.getLastStepVehicleNumber(secession_detector_id):
            vehicleflag = 1
        else:
            vehicleflag = 0
        
        command['CommandID'] = "0xF0060100"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['VehicleFlg'] = str(vehicleflag)
        # valuelist.append(value)
        command['Value'] = value

        return command
        
    # 工事帯内車両なし操作
    def construction_vehicle_none_operation(self, nodeID):
        command = {}     # 送信データの雛形
        # value = {}
        # valuelist = []
        timeStamp = self.get_time()
        command['CommandID'] = "0x00010140"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)

        return command
        
    # 緊急停止解除操作
    def collision_caution_release_operation(self, nodeID):
        command = {}     # 送信データの雛形
        value = {}
        # valuelist = []
        timeStamp = self.get_time()
        command['CommandID'] = "0x00010700"
        command['EventID'] = "_".join([nodeID, str(timeStamp)])
        command['TimeStamp'] = str(timeStamp)
        value['Operation'] = "0"
        command['Value'] = value

        return command
        
    def get_vehicle_type(self, vehicleID):
        typeID = traci.vehicle.getTypeID(vehicleID)
        #print(typeID)
        if vehicleID == None:           # 対象なし
            vehicleType = -1
        elif typeID == "normal":           # 普通車
            vehicleType = 0
        elif typeID == "track" or typeID == "large":         # トラック
            vehicleType = 1
        elif typeID == "bus":           # バス   
            vehicleType = 2
        elif typeID == "bicycle":       # 2輪車
            vehicleType = 3
        elif typeID == "emargency":     # 緊急車両
            vehicleType = 4
        else:                           # 異常値
            vehicleType = -9
        return vehicleType

    # 設定ファイル読込
    def setting_read(self, file_name):
        file_open = open(file_name, "r")
        settings = json.load(file_open)

        return settings
        
    # シミュレーション設定
    def simulation_setting(self):
        return True

    # 枝道設定
    def set_branch_info(self):
        # 枝道がない場合、空のリストを返す
        if not self.settings["NODES_APPROACH_DETECTOR"] or not self.settings["NODES_SECESSION_DETECTOR"]:
            return []
        
        branch_info = []
        for i, approach_detector_id_list in enumerate(self.settings["NODES_APPROACH_DETECTOR"].split(" ")):
            secession_detector_id_list = self.settings["NODES_SECESSION_DETECTOR"].split(" ")[i]
            node_id = "BoxPC" + str(i + 3).zfill(2)
            info = {}
            info["node_id"] = node_id
            approach_detector_id = approach_detector_id_list.split(",")
            secession_detector_id = secession_detector_id_list.split(",")
            approach_info = {}
            approach_info["id"] = approach_detector_id
            info["approach"] = approach_info
            secession_info = {}
            secession_info["id"] = secession_detector_id
            info["secession"] = secession_info
            branch_info.append(info)
        # print(branch_info)
        return branch_info

    # 枝道情報取得
    def get_branch_data(self, node_id):
        branch_data = None
        for data in self.branch_info:
            if data["node_id"] == node_id:
                branch_data = data
                break

        return branch_data

    def set_congestion_vehicle_number(self):
        result = {}
        result["straight"] = -1
        result["regulation"] = -1
        for branch in self.branch_info:
            result[branch["node_id"]] = -1

        return result
        
    # バウンディングボックス計算
    def bounding_box(self, detection, detection_zone, vehicle_position):

        # 工事帯からの距離に合わせてバウンディングボックスのサイズをセット(工事帯から遠いほど小さく、近いほど大きく)
        detection_vehicle_position = (detection_zone - vehicle_position) / detection_zone
        bounding_box_size = [self.bounding_box_max[0] - self.bounding_box_min[0], self.bounding_box_max[1] - self.bounding_box_min[1]]
        bounding_box_width = self.bounding_box_min[0] + bounding_box_size[0] * detection_vehicle_position
        bounding_box_height = self.bounding_box_min[1] + bounding_box_size[1] * detection_vehicle_position

        # 工事帯からの距離に合わせてバウンディングボックスの位置をセット(工事帯から遠いほど上部、近いほど下部)
        bounding_box_area = self.image_size[1] - self.bounding_box_min[1]
        bounding_box_bottom = int(self.bounding_box_min[1] + bounding_box_area * detection_vehicle_position)
        bounding_box_top = int(bounding_box_bottom - bounding_box_height)
        bounding_box_left = 0
        bounding_box_right = 0
        NP_bounding_box_center = 0
        # 接近車両か離脱車両か判断
        if detection:
            # 接近車両の場合、画像サイズの右半分をセット
            bounding_box_left = int(self.image_size[0] / 2)
            bounding_box_right = int(bounding_box_left + bounding_box_width)
            NP_bounding_box_center =  bounding_box_left + bounding_box_width / 2
        else:
            # 離脱車両の場合、画像サイズの左半分をセット
            bounding_box_right = int(self.image_size[0] / 2)
            bounding_box_left = int(bounding_box_right - bounding_box_width)
            NP_bounding_box_center =  bounding_box_right - bounding_box_width / 2

        # ナンバープレートのバウンディングボックス位置セット(車両のバウンディングボックスを基準とし、水平方向を中心、垂直方向は下あたり)
        NP_bounding_box_height = bounding_box_height * (8 / 100)    # ナンバープレートのバウンディングボックスの高さ
        NP_bounding_box_width = NP_bounding_box_height * 2          # ナンバープレートのバウンディングボックスの横幅
        NP_bounding_box_bottom = int(bounding_box_bottom - NP_bounding_box_height)   #
        NP_bounding_box_top = int(NP_bounding_box_bottom - NP_bounding_box_height)
        NP_bounding_box_left = int(NP_bounding_box_center - NP_bounding_box_width / 2)
        NP_bounding_box_right = int(NP_bounding_box_center + NP_bounding_box_width / 2)

        # 車両のバウンディングボックス位置座標リスト
        bounding_box_position = [bounding_box_left, bounding_box_top, bounding_box_right, bounding_box_bottom]
        # ナンバープレートのバウンディングボックス位置座標リスト
        NP_bounding_box_position = [NP_bounding_box_left, NP_bounding_box_top, NP_bounding_box_right, NP_bounding_box_bottom]

        # print("imageSize: " + image_size_())
        # print("vehiclePosition: " + str(detection_zone - vehicle_position))
        # print("boundingBox: " + str(bounding_box_position))
        # print("NPboundingBox: " + str(NP_bounding_box_position))

        return bounding_box_position, NP_bounding_box_position


    def set_vehicle_position(self, detection, detector_id, vehicle_id, vehicle_target_flag):
        detector_length = traci.lanearea.getLength(detector_id) # 検出器の長さ
        vehicle_length = traci.vehicle.getLength(vehicle_id) # 車両の長さ
        dist_to_detector_end = traci.lanearea.getVehicleDistToDetectorEnd(detector_id, vehicle_id)

        # if detector_id == self.settings["STRAIGHT_APPROACH_DETECTOR"]:
        #     tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        # elif detector_id == self.settings["REGULATION_APPROACH_DETECTOR"]:
        #     tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]
        # for i, secession_detector_id in enumerate(self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")):
        #     if detector_id == secession_detector_id:
        #         tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        # for i, secession_detector_id in enumerate(self.settings["REGULATION_SECESSION_DETECTOR"].split(",")):
        #     if detector_id == secession_detector_id:
        #         tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]

        # if detector_lane_list:

        # elif detector_id in self.settings["REGULATION_SECESSION_DETECTOR"].split(","):
        #     detector_lane_list = self.settings["REGULATION_SECESSION_DETECTOR_LANES"].split(",")
        #     tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]

        # 車両が検出器の設置してあるレーンにいない場合Noneを返す
        # if traci.vehicle.getLaneID(vehicle_id) != lane_id:
        #     return None

        # 対象車両が車両全体を含むかどうか
        if vehicle_target_flag:
            vehicle_magnification = detector_length / (detector_length - vehicle_length)
        else:
            vehicle_magnification = 1

        # 接近か離脱か
        if detection:
            vehicle_position = (dist_to_detector_end) * vehicle_magnification
        else:
            vehicle_position = (detector_length - dist_to_detector_end) * vehicle_magnification

        # 車両全体が検出器に入っていない場合Noneを返す(車両の一部だけ検出器に入っている場合はNG)
        if vehicle_position > detector_length or vehicle_position < 0:
            return None

        # print(detector_id)
        # print(vehicle_position)

        # print("detector: " + str(detector_length))
        # print("lane~detector: " + str(lane_detector_length))
        # print("lane: " + str(lane_length))
        # print("vehicle: " + str(vehicle_position))
        # print(vehicle_position)
        # print(detector_lane_list)

        return vehicle_position

    # 接続レーン取得
    def get_next_lane(self, lane_id):
        # next_lane_data = []
        lane_links = traci.lane.getLinks(lane_id)
        for lane in lane_links[0]:
            # lane_data = []
            # if lane not in traci.lane.getIDList() or lane in next_lane_data[0]:
            if lane not in traci.lane.getIDList():
                continue
            # print(lane)
            # print(traci.lane.getLength(lane))
            # print(traci.lane.getEdgeID(lane))
            # print(traci.lane.getLinks(lane))
        # return next_lane_data

    def get_lane_list(self, lane_id):
        next_lane_id = lane_id
        lane_list = []
        while next_lane_id != "":
            lane_links = traci.lane.getLinks(next_lane_id)
            if lane_links == []:
                break
            for lane in lane_links[0]:
                if lane not in traci.lane.getIDList():
                    continue
                next_lane_id = lane

            if next_lane_id not in lane_list:
                lane_list.append(next_lane_id)

        return lane_list
        
    # 検出対象のレーン判断
    def detection_lane_judge(self, detector_id, tls_id, vehicle_id, detection):
        if tls_id == "":
            return True

        detection_lane_ids = list(traci.lanearea.getLaneIDs(detector_id)) # 検出範囲内すべてのレーンID
        if detection:
            detection_lane_ids.reverse()
        # print(detection_lane_ids)
        # print(vehicle_id)
        vehicle_lane_id = traci.vehicle.getLaneID(vehicle_id)

        for lane in detection_lane_ids:
            edge_id = traci.lane.getEdgeID(lane)
            # print(edge_id, tls_id)
            if tls_id in edge_id:
                return False

            # print(vehicle_lane_id, lane)
            if vehicle_lane_id == lane:
                return True
            
        return False

    # 検出判定
    def detection_judge(self, detection_rate):
        return random.choices([True, False], weights = [detection_rate,100 - detection_rate])[0]
        # print()

    # ナンバー検出判定
    def number_detection_judge(self, detection_rate, number):
        judge = random.choices([True, False], weights = [detection_rate,100 - detection_rate])[0]
        if judge:
            number = number
        else:
            number = self.set_false_positive_number(number)

        return number
            
        # print()

    # 誤検出ナンバー作成
    def set_false_positive_number(self, number):
        if len(number) == 1:
            number_list = list(range(10))
            number_list.remove(0)
            number_list.remove(int(number))
            number = str(random.choice(number_list))
        else:
            n_list = list(number)
            false_positive_index = random.sample(list(range(len(number))), 2)
            for i in false_positive_index:
                number_list = list(range(10))
                # if i == 0:
                #     number_list.remove(0)
                number_list.remove(int(n_list[i]))
                n_list[i] = str(random.choice(number_list))

            number = "".join(n_list)
        
        return number
                
    # 車両情報割り当て
    def set_vehicle_info(self):
        vehicleIDs = traci.vehicle.getIDList()
        for id in vehicleIDs:
            # test_number = traci.vehicle.getPersonIDList(id)
            # print("number:" + str(test_number))
            dictId = {}
            # 辞書にキーがない場合キーと値を追加
            if str(id) not in self.vehicle_info.keys():
                number = traci.vehicle.getLicenseNumber(id)
                number = str(number).zfill(4)
                dictId["Number"] = str(number)
                dictId["EntryFlag"] = "False"
                dictId["BreakawayFlag"] = "False"
                dictId["VehicleType"] = str(self.get_vehicle_type(id))
                self.vehicle_info[str(id)] = dictId
                #print(self.vehicle_info)

            # 各検出判定
            # 車列検出判定
            self.vehicle_info[str(id)]["CarsDetectionFlag"] = self.detection_judge(100)
            self.vehicle_info[str(id)]["VehicleRecognitionFlag"] = self.detection_judge(100)
            self.vehicle_info[str(id)]["SpeedDistanceFlag"] = self.detection_judge(100)
            self.vehicle_info[str(id)]["OutputNumber"] = self.number_detection_judge(int(self.settings["LICENSE_PLATE_DETECTION_RATE"]), self.vehicle_info[str(id)]["Number"])
            

    # データ送信時の画像サイズ成形
    def set_image_size(self):
        return str(self.image_size[0]) + "x" + str(self.image_size[1])

    def set_command(self, command_dict):
        command_str = str(command_dict)
        return command_str.replace('\'', '"')

    # 工事帯初期誘導状態セット
    def set_traffic_state_signal(self, traffic_guide_id):
        tls_state = traci.trafficlight.getRedYellowGreenState(traffic_guide_id)
        
        tls_state_list = {}
        tls_state_list["red"] = tls_state
        tls_state_list["yellow"] = tls_state.replace('r', 'y')
        tls_state_list["green"] = tls_state.replace('r', 'G')
        tls_state_list["state"] = "red"

        return tls_state_list

    # def traffic_state_convert(color):
        

    # 誘導状態更新
    def set_traffic_state(self, lane_type, traffic_state):
        # ストレート、規制、枝道のどれか判定
        if lane_type == "0":
            lane = "straight"
        if lane_type == "1":
            lane = "regulation"
        if lane_type >= "2":
            lane = "node" + str(int(lane_type) - 1)
            # traffic_state = traci.trafficlight.getPhaseName(self.straight_traffic_guide_id)
        color_state = self.tls_state_list[lane]["state"]

        # 誘導状態の情報をセット
        if traffic_state == "0" or traffic_state == "2":
            traffic_state_str = "red"
        if traffic_state == "1":
            traffic_state_str = "green"
        if traffic_state == "3":
            traffic_state_str = "yellow"

        # 誘導状態が切り替わっているか判定
        if color_state != traffic_state_str:
            # SUMO上の工事帯信号を変更
            traci.trafficlight.setRedYellowGreenState(self.tls_state_list[lane]["id"], self.tls_state_list[lane][traffic_state_str])
            self.tls_state_list[lane]["state"] = traffic_state_str

            # 誘導信号変化履歴書き込み
            self.traffic_guide_change_history()
        
        return

    # タイムアウト処理
    def time_out(self):
        # タイムアウト発生中か判定
        if not self.time_out_flag:
            return

        construction_vehicle_number = self.get_inside_car_number()
        if construction_vehicle_number > 0:
            return

        # 工事帯内車両なしとなった時間が格納済みか判定
        if self.construction_vehicle_time == None:
            self.construction_vehicle_time = datetime.datetime.now()
        
        # 工事帯内車両なしとなってから5秒経過しているか判定
        passed_time = datetime.datetime.now() - self.construction_vehicle_time
        if passed_time.seconds >= 5:
            # 工事帯内車両なし操作
            # print("タイムアウト解除")
            result = self.construction_vehicle_none_operation("Tablet01")
            self.send(self.set_command(result))
            self.sumo_log.info("-----------------------------タイムアウト解除-----------------------------")
            self.sumo_log.info(result)
            if self.collision_caution:
                result = self.collision_caution_release_operation("Tablet01")
                self.send(self.set_command(result))
                self.sumo_log.info("-----------------------------緊急停止解除-----------------------------")
                self.sumo_log.info(result)
                self.collision_caution = False
            self.construction_vehicle_time = None
            self.time_out_flag = False
            self.time_out_history_flag = False

        return

    # 誘導状態更新
    def lane_state_update(self, value_list):
        for value in value_list:
            self.set_traffic_state(value["LaneKind"], value["TrafficGuideState"])
        
        return

    # タイムアウト発生処理
    def construction_passing_state_update(self, value):
        if value["TimeOutFlg"] == "1":
            self.time_out_flag = True
            self.time_out_side = value["TimeOutSide"]
            self.sumo_log.info("-----------------------------タイムアウト発生-----------------------------")
            # self.sumo_log.info(result)

        return

    # 衝突チェック
    def check_collision_caution(self, values):
        if self.collision_caution:
            return
        
        for value in values:
            if int(value["CollisionCaution"]) == 1:
                self.sumo_log.info("-----------------------------緊急停止発生-----------------------------")
                self.collision_caution = True

        return

    # 先頭車両状態セット
    def set_leading_vehicle_info(self):
        result = {}
        # ストレート
        result["straight"] = -1
        # 規制
        result["regulation"] = -1
        # 枝道
        if len(self.nodes_traffic_guide_id) > 0 and '' not in self.nodes_traffic_guide_id:
            for i, node_traffic_guide_id in enumerate(self.nodes_traffic_guide_id):
                result_key = "node" + str(i + 1)
                result[result_key] = -1

        return result

    # 誘導信号変化履歴出力
    def traffic_guide_change_history(self):
        file_name = "guideTrafficLight.txt"
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        # if sim_time_stamp == self.guide_traffic_light_sim_time:
        #     return

        output_data = [str(time_stamp), sim_time_stamp]
        output_data_ = []
        # 誘導信号リストをループ
        for key, value in self.tls_state_list.items():
            # print(value)
            state = -1
            if value["state"] == "red":
                state = 0
            elif value["state"] == "green":
                state = 1
            elif value["state"] == "yellow":
                state = 2
            output_data.append(str(state))
            output_data_.append(str(state))
        
        if self.guide_traffic_light == output_data_:
            return
        self.guide_traffic_light = output_data_
        print(file_name)
        print(output_data)
        self.GuideTrafficLight(file_name, output_data)

    # 先頭車両接近・停止履歴出力
    def output_leading_vehicle_state_history(self):
        output_flg = False
        detector_id = None
        detection_range = 70   # 検出範囲(m)
        # 車線数分ループ
        for key, value in self.leading_vehicle_info.items():
            if key == "straight":
                detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
                lane_kind = 1
            elif key == "regulation":
                detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
                lane_kind = 2
            else:
                node_num = int(key[4:])
                detector_id = self.settings["NODES_APPROACH_DETECTOR"].split(",")[node_num - 1]
                lane_kind = node_num + 2

            if not detector_id:
                continue

            # 先頭車両ID取得
            leading_vehicle_id = self.get_leading_vehicle(detector_id, True)
            if not leading_vehicle_id:
                continue

            leading_vehicle_position = self.set_vehicle_position(True, detector_id, leading_vehicle_id, False)
            leading_vehicle_speed = traci.vehicle.getSpeed(leading_vehicle_id) * 3600 / 1000
            leading_vehicle_state = self.leading_vehicle_jugde(leading_vehicle_speed)
            number = self.vehicle_info[leading_vehicle_id]["Number"]

            # 先頭車両が検出範囲外の場合車両なしの情報をセット
            if leading_vehicle_position >= detection_range or leading_vehicle_position < 0:
                leading_vehicle_state = 0
                number = ""
                leading_vehicle_position = ""
                leading_vehicle_speed = ""
            

            # 先頭車両状態が切り替わったか判定
            if value == leading_vehicle_state:
                continue
            
            self.leading_vehicle_info[key] = leading_vehicle_state

            # ファイル書き込み
            file_name = "leadingCar_%s.txt" % (str(lane_kind))
            time_stamp = self.get_time()
            sim_time_stamp = self.get_sim_time()
            output_data = [str(time_stamp), sim_time_stamp, str(lane_kind), str(leading_vehicle_state), number, str(leading_vehicle_position), str(leading_vehicle_speed)]
            print(file_name)
            print(output_data)
            self.LeadingCar_LaneNumber(file_name, output_data)

    # 先頭車両状態判断
    def leading_vehicle_jugde(self, vehicle_speed):
        # detection_range = 70   # 検出範囲(m)
        vehicle_state = 0
        if vehicle_speed >= 5:
            vehicle_state = 1
        else:
            vehicle_state = 2
        
        return vehicle_state

    # 先頭車両ID取得
    def get_leading_vehicle(self, detector_id, detection = True):
        # 検出器上のすべての車両ID取得
        vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)

        leading_vehicle_id = None   # 先頭車両ID格納変数
        for vehicle_id in vehicle_ids:
            vehicle_position = self.set_vehicle_position(detection, detector_id, vehicle_id, True)
            if not vehicle_position:
                continue
            # 1台目の場合、先頭車両に設定して次の車両
            if not leading_vehicle_id:
                leading_vehicle_id = vehicle_id
                continue
            
            leading_vehicle_position = self.set_vehicle_position(detection, detector_id, leading_vehicle_id, True)

            # 先頭車両を更新
            if vehicle_position < leading_vehicle_position:
                leading_vehicle_id = vehicle_id

        return leading_vehicle_id

    # 工事帯内車両台数取得
    def get_inside_car_number(self):
        inside_car_number = 0
        # 工事帯検出器があるか判定
        straight_detector = self.settings["STRAIGHT_CONSTRUCTION_BAND_DETECTOR"]
        regulation_detector = self.settings["REGULATION_CONSTRUCTION_BAND_DETECTOR"]
        if straight_detector == "" or regulation_detector == "":
            return None

        inside_car_number = traci.lanearea.getLastStepVehicleNumber(straight_detector) + traci.lanearea.getLastStepVehicleNumber(regulation_detector)
        # print(inside_car_number)

        return inside_car_number
        # else:


    # 工事帯内車両台数履歴
    def inside_car_number_history(self):
        inside_car_number = self.get_inside_car_number()
        if self.inside_car_number == inside_car_number:
            return False
        
        file_name = "insideCar.txt"
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        output_data = [str(time_stamp), sim_time_stamp, str(inside_car_number)]
        self.inside_car_number = inside_car_number
        print(file_name)
        print(output_data)
        self.InsideCar(file_name, output_data)
        return True

    # 滞留発生履歴
    def residence_occurred_history(self):
        # ストレート側
        tls_id = self.settings["STRAIGHT_TRAFFIC_LIGHT"]
        detector_id_list = self.settings["STRAIGHT_SECESSION_DETECTOR"].split(",")
        if self.residence_occurred(tls_id, detector_id_list, "straight"):
            file_name = "stayCar_%s.txt" % ("1")
            time_stamp = self.get_time()
            sim_time_stamp = self.get_sim_time()
            lane_kind = 1
            residence_vehicle_number = self.residence_vehicle_number["straight"]
            output_data = [str(time_stamp), str(sim_time_stamp), str(lane_kind), str(residence_vehicle_number)]
            # print(file_name)
            # print(output_data)
            self.StayCar_LaneNumber(file_name, output_data)

        # 規制側
        tls_id = self.settings["REGULATION_TRAFFIC_LIGHT"]
        detector_id_list = self.settings["REGULATION_SECESSION_DETECTOR"].split(",")
        if self.residence_occurred(tls_id, detector_id_list, "regulation"):
            file_name = "stayCar_%s.txt" % ("2")
            time_stamp = self.get_time()
            sim_time_stamp = self.get_sim_time()
            lane_kind = 2
            residence_vehicle_number = self.residence_vehicle_number["regulation"]
            output_data = [str(time_stamp), str(sim_time_stamp), str(lane_kind), str(residence_vehicle_number)]
            # print(file_name)
            # print(output_data)
            self.StayCar_LaneNumber(file_name, output_data)


    # 滞留発生
    def residence_occurred(self, tls_id, detector_id_list, lane_kind):
        # traffic_light_list = [self.settings["STRAIGHT_TRAFFIC_LIGHT"], self.settings["REGULATION_TRAFFIC_LIGHT"]]
        residence_vehicle_number = 0
        if tls_id == "":
            return False

        for detector_id in detector_id_list:
            # 検出器内のすべての車両取得
            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
            for vehicle_id in vehicle_ids:
                # 検出範囲内か判断
                if not self.detection_lane_judge(detector_id, tls_id, vehicle_id, False):
                    continue

                vehicle_speed = traci.vehicle.getSpeed(vehicle_id) * 3600 / 1000
                # 停止中か判断
                if vehicle_speed > 0:
                    continue

                residence_vehicle_number += 1
        
        # print(residence_vehicle_number)
        if residence_vehicle_number == self.residence_vehicle_number[lane_kind]:
            return False
        
        self.residence_vehicle_number[lane_kind] = residence_vehicle_number
        return True

    def traffic_light_history(self, lane_kind, color_str):
        if color_str == "Green":
            color = 1
        elif color_str == "Yellow":
            color = 2
        elif color_str == "Red":
            color = 0
        else:
            return
        file_name = "nearbyTrafficLight_%s.txt" % (str(lane_kind))
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        output_data = [str(time_stamp), str(sim_time_stamp), str(lane_kind), str(color)]
        print(file_name)
        print(output_data)
        self.NearbyTrafficLight_LaneNumber(file_name, output_data)

    # 渋滞発生履歴出力
    def congestion_history(self):
        output_flg = False
        detector_id = None
        detection_range = 70   # 検出範囲(m)
        # 車線数分ループ
        for key, value in self.congestion_vehicle_number.items():
            congestion_vehicle_number = 0
            if key == "straight":
                detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
                lane_kind = 1
            elif key == "regulation":
                detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
                lane_kind = 2
            else:
                node_num = int(key[4:])
                detector_id = self.settings["NODES_APPROACH_DETECTOR"].split(",")[node_num - 1]
                lane_kind = node_num + 2

            if not detector_id:
                continue

            # 検出器上のすべての車両ID取得
            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)

            for vehicle_id in vehicle_ids:
                vehicle_position = self.set_vehicle_position(True, detector_id, vehicle_id, False)
                if not vehicle_position:
                    continue

                vehicle_speed = traci.vehicle.getSpeed(vehicle_id) * 3600 / 1000
                # 範囲内かつ速度が5km以下か判定
                if vehicle_position <= detection_range and vehicle_speed <= 5:
                    congestion_vehicle_number += 1

            # 先頭車両状態が切り替わったか判定
            if value == congestion_vehicle_number:
                continue
            
            self.congestion_vehicle_number[key] = congestion_vehicle_number

            # ファイル書き込み
            file_name = "trafficJam_%s.txt" % (str(lane_kind))
            time_stamp = self.get_time()
            sim_time_stamp = self.get_sim_time()
            output_data = [str(time_stamp), sim_time_stamp, str(lane_kind), str(congestion_vehicle_number)]
            print(file_name)
            print(output_data)
            self.TrafficJam_LaneNumber(file_name, output_data)

    # シミュレーション開始・終了時刻出力
    def sim_start_end_time(self):
        file_name = "startTimestamp.txt"
        time_stamp = self.get_time()
        output_data = [str(time_stamp)]
        print(file_name)
        print(output_data)
        self.startTimestamp(file_name, output_data)


    # タイムアウト発生履歴出力
    def time_out_history(self):
        if not self.time_out_flag or self.time_out_history_flag:
            return

        file_name = "timeout.txt"
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        lane_kind = self.time_out_side
        output_data = [str(time_stamp), str(sim_time_stamp), str(lane_kind)]
        print(file_name)
        print(output_data)
        self.time_out_history_flag = True
        self.timeout(file_name, output_data)

    # 工事帯通過車両履歴出力
    def passing_vehicle_history(self, lane_kind, vehicle_id):
        file_name = "passingCar_%s.txt" % (str(lane_kind))
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        number = self.vehicle_info[str(vehicle_id)]["Number"]
        speed = traci.vehicle.getSpeed(vehicle_id) * 3600 / 1000
        output_data = [str(time_stamp), str(sim_time_stamp), str(lane_kind), number, str(speed)]
        print(file_name)
        print(output_data)
        self.passingCar_LaneNumber(file_name, output_data)

    # すすめ可能からの待ち時間履歴出力
    def waiting_time_history(self):
        # すすめ可能か判断
        go_possible_flag = self.go_possible_judge()

        if self.go_possible_flag == go_possible_flag:
            return

        self.go_possible_flag = go_possible_flag
        if go_possible_flag:
            self.go_possible_time = self.get_time()
            self.go_possible_sim_time = self.get_sim_time()
            return
        else:
            go_change_time = self.get_time()
            go_change_sim_time = self.get_sim_time()
            # すすめとなった車線取得
            lane_kind = -1
            for key, value in self.tls_state_list.items():
                if value["state"] != "green":
                    continue
                if key == "straight":
                    lane_kind = 1
                elif key == "regulation":
                    lane_kind = 2
                else:
                    node_num = int(key[4:])
                    lane_kind = node_num + 2

            if lane_kind == -1:
                return

        # すすめ時間
        file_name = "waitTime.txt"
        time_stamp = self.go_possible_time
        sim_time_stamp = self.go_possible_sim_time
        wait_time = int(go_change_sim_time) - int(sim_time_stamp)
        output_data = [str(time_stamp), str(go_change_time), str(sim_time_stamp), str(go_change_sim_time), str(lane_kind), str(wait_time)]
        print(file_name)
        print(output_data)
        self.waitTime(file_name, output_data)


    # すすめ可能か判断
    def go_possible_judge(self):
        detection_range = 70
        # 工事帯内に車両なしか判断
        if self.get_inside_car_number() != 0:
            return False

        # 全車線の誘導状態がとまれか判断
        for key, value in self.tls_state_list.items():
            if value["state"] != "red":
                return False
        
        # 停止車両が存在するか確認
        # 車線数分ループ
        for key, value in self.congestion_vehicle_number.items():
            congestion_vehicle_number = 0
            if key == "straight":
                detector_id = self.settings["STRAIGHT_APPROACH_DETECTOR"]
                lane_kind = 1
            elif key == "regulation":
                detector_id = self.settings["REGULATION_APPROACH_DETECTOR"]
                lane_kind = 2
            else:
                node_num = int(key[4:])
                detector_id = self.settings["NODES_APPROACH_DETECTOR"].split(",")[node_num - 1]
                lane_kind = node_num + 2

            vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
            for vehicle_id in vehicle_ids:
                vehicle_position = self.set_vehicle_position(True, detector_id, vehicle_id, False)
                if not vehicle_position:
                    continue
                vehicle_speed = traci.vehicle.getSpeed(vehicle_id) * 3600 / 1000
                
                # 範囲内かつ速度が5km以下か判定
                if vehicle_position <= detection_range and vehicle_speed <= 5:
                    return True

        return False

    # 衝突発生履歴
    def collision_history_output(self, s_vehicle_id, r_vehicle_id):
        straight_detector_id = self.settings["STRAIGHT_CONSTRUCTION_BAND_DETECTOR"]
        regulation_detector_id = self.settings["REGULATION_CONSTRUCTION_BAND_DETECTOR"]
        file_name = "detCADanger.txt"
        time_stamp = self.get_time()
        sim_time_stamp = self.get_sim_time()
        
        # ストレート側の車両
        straight_data = self.collision_history_detection_data(straight_detector_id, 1, s_vehicle_id)
        # 規制側の車両
        regulation_data = self.collision_history_detection_data(regulation_detector_id, 2, r_vehicle_id)

        output_data = [str(sim_time_stamp), str(time_stamp), straight_data, regulation_data]
        # 枝道分のデータ格納
        # output_data.append("branch")
        print(file_name)
        print(output_data)
        self.detCADanger(file_name, output_data)

    # 衝突情報取得
    def collision_history_detection_data(self, detector_id, lane_kind, vehicle_id):
        if lane_kind == 1:
            det_id = "0021"
            detection = True
        elif lane_kind == 2:
            det_id = "0022"
            detection = False
        else:
            det_id = "0023"
        
        vehicle_ids = traci.lanearea.getLastStepVehicleIDs(detector_id)
        construction_vehicle_number = str(len(vehicle_ids))
        # leading_vehicle_id = self.get_leading_vehicle(detector_id, detection)
        # print(leading_vehicle_id)
        leading_vehicle_distance = str(traci.lanearea.getVehicleDistToDetectorEnd(detector_id, vehicle_id))
        leading_vehicle_speed = str(traci.vehicle.getSpeed(vehicle_id) * 3600 / 1000)
        construction_vehicle_id = "{" + ",".join(vehicle_ids) + "}"
        result = "{%s, %s, %s, %s, %s}" % (det_id, construction_vehicle_number, leading_vehicle_distance, leading_vehicle_speed, construction_vehicle_id)
        return result

    def GuideTrafficLight(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, system_state" + "\n")
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def LeadingCar_LaneNumber(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, node_no, status, car_number, distance, speed" + "\n")
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def InsideCar(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, inside_car_count" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def StayCar_LaneNumber(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, node_no, vehicle_count" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def NearbyTrafficLight_LaneNumber(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, node_no, color" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def TrafficJam_LaneNumber(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, node_no, vehicle_count" + "\n")
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def startTimestamp(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            # if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                # f.write("#time, sim_time, node_no, vehicle_count" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def timeout(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#start_time, start_sim_time, TimeOutSide" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def passingCar_LaneNumber(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#time, sim_time, node_no, vehicle_count" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def waitTime(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write("#start_time, end_time, start_sim_time, end_sim_time, node_no, wait_time" + "\n")  
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def detCADanger(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            if os.path.getsize(os.path.join(self.output_directory, fileName)) == 0:
                f.write('#"time", "sim_time", "{unit_id, vehicle_count, head_vehicle_distance, vehicle_speed, {vehicle_id}, ... }", ...' + "\n")
            if len(datalist) != 0:
                f.write(', '.join(datalist) + "\n")

    def write(self, fileName, datalist):
        with open(os.path.join(self.output_directory, fileName), 'a') as f:
            f.write(', '.join(datalist) + "\n")

    def output_header(self):
        datalist = []
        self.GuideTrafficLight("guideTrafficLight.txt", "")
        for key, value in self.leading_vehicle_info.items():
            if key == "straight":
                lane_kind = 1
            elif key == "regulation":
                lane_kind = 2
            else:
                node_num = int(key[4:])
                lane_kind = node_num + 2

            # file_name = "leadingCar_%s.txt" % (str(lane_kind))
            self.LeadingCar_LaneNumber("leadingCar_%s.txt" % (str(lane_kind)), datalist)
            self.TrafficJam_LaneNumber("trafficJam_%s.txt" % (str(lane_kind)), datalist)
        self.InsideCar("insideCar.txt", datalist)
        self.StayCar_LaneNumber("stayCar_%s.txt" % ("1"), datalist)
        self.StayCar_LaneNumber("stayCar_%s.txt" % ("2"), datalist)
        self.NearbyTrafficLight_LaneNumber("nearbyTrafficLight_%s.txt" % ("1"), datalist)
        self.NearbyTrafficLight_LaneNumber("nearbyTrafficLight_%s.txt" % ("2"), datalist)
        self.timeout("timeout.txt", datalist)
        self.passingCar_LaneNumber("passingCar_%s.txt" % ("1"), datalist)
        self.passingCar_LaneNumber("passingCar_%s.txt" % ("2"), datalist)
        self.waitTime("waitTime.txt", datalist)
        self.detCADanger("detCADanger.txt", datalist)



    def recv(self):
        self.api_run()
        # print("test")

    # HTTP送信
    def send(self, js):
        if js == "None":
            return
        headers = {'Content-Type': 'application/json',}
        req = urllib.request.Request(self.url,js.encode(),headers)
        try:
            self.sumo_log.info("SendTime: " + str(datetime.datetime.now()))
            with urllib.request.urlopen(req,timeout=int(self.send_time_out)) as res:
                body = res.read().decode("utf-8")
        except urllib.error.URLError:
            self.sumo_log.info("エラー：設定されているURLへの接続に失敗しました。")
        except socket.timeout:
            self.sumo_log.info("エラー：コマンド送信中にタイムアウトしました。")

    # 
    def api_run(self):
        return self.api.run(host=self.host, port=self.port, debug=False)

    # POSTリクエストの処理
    # @self.api.route('/', methods=['POST'])
    def post_sim_recv(self):
        dt_now = datetime.datetime.now()

        # print("\n==========================================================\n")
        # print("Time: {}\n".format(dt_now))
        # print("RowData: {}\n".format(request.data.decode()))
        # print(data)

        data = json.loads(request.data.decode())
        command_type = 0
        command_id = ""

        for key in data:
            if key != "Value":
                # print("{: <20}: {}".format(key, data[key]))
                if key == "CommandID":
                    command_id = data[key]
                    if data[key][2] == 'F':
                        command_type = 1
            else:
                if isinstance(data[key], list):
                    title = "Value[idx]"
                    values = data[key]
                else:
                    title = "Value (not array)"
                    values = [data[key]]

                # for idx, value in enumerate(values):
                    # print("\n*** {} ***".format(title.replace("idx", str(idx))))

                    # for value_key in value:
                        # print("{: <20}: {}".format(value_key, value[value_key]))
        # print("\n==========================================================\n")

        if command_type == 0:
            result = jsonify({"Message": "Reception success"})
        else:
            result = ''
        
        # コマンドID判定
        if command_id == "0x00000070":
            # 車線状態更新要求を受信した際の処理
            self.lane_state_update(values)

        elif command_id == "0x00000080":
            # 工事帯通過状態更新要求を受信した際の処理
            self.construction_passing_state_update(values[0])

        elif command_id == "0x00000060":
            # システム判断状態更新要求を受信した際の処理
            self.check_collision_caution(values)

        res = make_response(result)
        return res


    # @self.api.errorhandler(404)
    def not_found(error):
        return make_response(jsonify({'error': 'Not found'}), 404)
            

# this is the main entry point of this script
if __name__ == "__main__":
    try:
        sumoSim = SumoSim()
    except KeyboardInterrupt:
        print("シミュレーション終了")
        sys.exit() 
    except traci.exceptions.FatalTraCIError:
        print("シミュレーション終了")
        sys.exit()
    # options = sumoSim.get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run

