/*
  Network settings of the MobaStation ESP firmware.

  Saved in the web interface (LittleFS /net.json) they win over the defaults the Mega
  sends from MobaStationConfig.h. Without saved settings the defaults of the Mega are used.
*/
#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define NET_MODE_STATION 0      //only the router
#define NET_MODE_AP      1      //only the own access point
#define NET_MODE_HYBRID  2      //router, own access point while the router is not reachable

struct NetConfig {
    uint8_t mode = NET_MODE_HYBRID;
    String ssid;
    String pwd;
    String apSsid = "MobaStation";
    String apPwd = "MobaStation";
    bool dhcp = false;
    IPAddress ip = IPAddress(192, 168, 0, 111);
    IPAddress subnet = IPAddress(255, 255, 255, 0);
    IPAddress gateway = IPAddress(192, 168, 0, 1);
    uint16_t port = 21105;
    bool saved = false;         //from the web interface

    bool sameNetwork(const NetConfig &o) const {
        return mode == o.mode && ssid == o.ssid && pwd == o.pwd && apSsid == o.apSsid && apPwd == o.apPwd &&
               dhcp == o.dhcp && ip == o.ip && subnet == o.subnet && gateway == o.gateway && port == o.port;
    }
};

static const char *NET_FILE = "/net.json";

static void parseIp(JsonVariantConst v, IPAddress &ip){
    IPAddress tmp;
    if(v.is<const char*>() && tmp.fromString(v.as<const char*>())) ip = tmp;
}

//fields that are missing in the JSON keep their value, an empty password keeps the old one
static void configFromJson(NetConfig &c, JsonVariantConst j){
    if(j["mode"].is<int>()) c.mode = constrain(j["mode"].as<int>(), 0, 2);
    if(j["ssid"].is<const char*>()) c.ssid = j["ssid"].as<const char*>();
    if(j["pwd"].is<const char*>() && strlen(j["pwd"].as<const char*>()) > 0) c.pwd = j["pwd"].as<const char*>();
    if(j["apSsid"].is<const char*>() && strlen(j["apSsid"].as<const char*>()) > 0) c.apSsid = j["apSsid"].as<const char*>();
    if(j["apPwd"].is<const char*>() && strlen(j["apPwd"].as<const char*>()) > 0) c.apPwd = j["apPwd"].as<const char*>();
    if(j["dhcp"].is<bool>()) c.dhcp = j["dhcp"].as<bool>();
    parseIp(j["ip"], c.ip);
    parseIp(j["subnet"], c.subnet);
    parseIp(j["gateway"], c.gateway);
    if(j["port"].is<int>()) c.port = j["port"].as<int>();
}

static void configToJson(const NetConfig &c, JsonObject j, bool withPasswords){
    j["mode"] = c.mode;
    j["ssid"] = c.ssid;
    j["apSsid"] = c.apSsid;
    if(withPasswords){
        j["pwd"] = c.pwd;
        j["apPwd"] = c.apPwd;
    }
    else{
        j["pwdSet"] = c.pwd.length() > 0;
    }
    j["dhcp"] = c.dhcp;
    j["ip"] = c.ip.toString();
    j["subnet"] = c.subnet.toString();
    j["gateway"] = c.gateway.toString();
    j["port"] = c.port;
    j["saved"] = c.saved;
}

static bool loadConfig(NetConfig &c){
    File f = LittleFS.open(NET_FILE, "r");
    if(!f) return false;
    JsonDocument d;
    DeserializationError e = deserializeJson(d, f);
    f.close();
    if(e) return false;
    configFromJson(c, d.as<JsonVariantConst>());
    c.saved = true;
    return true;
}

static bool saveConfig(const NetConfig &c){
    File f = LittleFS.open(NET_FILE, "w");
    if(!f) return false;
    JsonDocument d;
    configToJson(c, d.to<JsonObject>(), true);
    serializeJson(d, f);
    f.close();
    return true;
}

static void clearConfig(){
    LittleFS.remove(NET_FILE);
}
