# Schlafüberwachung mit ESP32 & Raspberry Pi 
## Projektübersicht
Dieses Projekt beschreibt ein tragbares System zur **Überwachung des Schlafs** mithilfe eines **ESP32**, eines **Pulssensors** und eines **Gyroskops (MPU6050)**. Die Sensordaten werden während des Schlafs erfasst und über das **MQTT-Protokoll** an einen **Raspberry Pi** gesendet, wo sie verarbeitet und visualisiert werden:

Ziel ist es, Puls- und Bewegungsdaten auszuwerten, um Rückschlüsse auf Schlafphasen zu ziehen. 

--- 

## Systemarchitektur 

**ESP32** 
- Erfasst Sensordaten (Puls & Bewegung) 
- Sendet Daten per MQTT an den Raspberry Pi 

**Raspberry Pi** 
- Läuft als MQTT-Broker 
- MQTT-Client (Python) empfängt die Daten 
- Datenanalyse (FFT, Peak Detection, Curve Fitting) 
- Visualisierung der Ergebnisse

Die Kommunikation erfolgt im lokalen Netzwerk. 

---

## Hardware 
- ESP32 
- Pulssensor **HW-827** 
- Gyroskop **MPU6050** 
- Raspberry Pi (getestet mit Raspberry Pi 4B) 
- WLAN-Verbindung 

--- 

## Software-Voraussetzungen 
### ESP32 
- Arduino IDE oder PlatformIO 
- Bibliotheken: 
    - Adafruit MPU6050 
    - Adafruit Unified Sensor 
    - PubSubClient (MQTT) 
    - WiFi 
### Raspberry Pi 
- Raspberry Pi OS 
- Python 3 
- Python-Bibliotheken: 
    - `paho-mqtt` 
    - `numpy` 
    - `scipy` 
    - `matplotlib` 

---

## Installation
Installation der Python-Abhängigkeiten: 
```bash
pip3 install paho-mqtt numpy scipy matplotlib 
```

Auf dem Raspberry Pi **muss ein MQTT-Broker installiert und aktiv sein**, da der ESP32 seine Sensordaten dorthin sendet.

### Empfohlener Broker: Mosquitto Installation: 
```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
```
Broker aktivieren und starten:
```bash
sudo systemctl enable mosquitto
sudo systemctl start mosquitto 
```
- Offizielle Dokumentation: https://mosquitto.org/documentation/
- Hilfe bei der Installation von MQTT: https://randomnerdtutorials.com/how-to-install-mosquitto-broker-on-raspberry-pi/

In `esp_client/` muss anschließend eine Datei `secrets.h` angelegt werden, die folgende Makros enthält: `SSID` (WLAN-SSID), `PASSWORD` (WLAN-Passwort) und `MQTT_IP` (IP des MQTT-Brokers).

---

## MQTT-Topics
Der ESP32 veröffentlicht Daten unter folgenden Topics: 
- `esp32/pulse` 
Format: 
```timestamp,value```
- `esp32/movement`
Format:
```timestamp,value```
Der Raspberry-Pi-Client abonniert diese Topics und verarbeitet die eingehenden Daten.

---

## Datenverarbeitung

### Puls
- Mittelwertbildung zur Rauschreduktion
- Peak Detection mit `scipy` 
- FFT zur Frequenzanalyse
- Gaussian Curve Fit zur genaueren Herzfrequenzbestimmung

### Bewegung
- Gyroskopwerte (x, y, z) werden zu einem Intensitätswert aufsummiert
- Auswertung in festen 30-Sekunden-Intervallen

---

## Visualisierung

Es werden mehrere Diagramme erzeugt:
1. Rohdaten des Pulssensors (10-Sekunden-Fenster) 
2. FFT-Spektrum
3. Rohdaten des Gyroskops
4. Bewegungsintensität über die Zeit
5. Verlauf der Herzfrequenz

---

## Hinweise & Einschränkungen
- Der verwendete Pulssensor ist **nicht zuverlässig genug** für stabile Langzeitmessungen
- Die Ergebnisse sind **experimentell** und **nicht medizinisch verwertbar**
- Das Projekt dient ausschließlich Lern- und Demonstrationszwecken

---

## Mögliche Erweiterungen
- Einsatz eines zuverlässigeren Pulssensors
- Verwendung eines kleineren ESP32-Modells
- Zusätzliche Sensoren (z. B. SpO₂)
- Machine-Learning-Ansätze zur Schlafphasenerkennung
- Intelligenter Wecker basierend auf Leichtschlafphasen
