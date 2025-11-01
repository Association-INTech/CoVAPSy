#!/bin/bash

# Vérifie que bluetoothctl est dispo
if ! command -v bluetoothctl &>/dev/null; then
    echo "Erreur : bluetoothctl n'est pas installé."
    exit 2
fi

# Lecture du fichier ligne par ligne
while read -r line; do
    # Ignore les lignes vides ou commençant par #
    [[ -z "$line" || "$line" =~ ^# ]] && continue

    # Récupère le premier mot (adresse MAC)
    mac=$(echo "$line" | awk '{print $1}')

    echo "-----------------------------------------"
    echo "   Traitement de l'appareil : $mac"
    echo "-----------------------------------------"

    bluetoothctl remove "$mac"
    bluetoothctl -t 8 scan on &
    scan_pid=$!
    sleep 4
    kill "$scan_pid" 2>/dev/null

    bluetoothctl pair "$mac"
    sleep 1
    bluetoothctl trust "$mac"
    sleep 1
    bluetoothctl connect "$mac"

    echo
done < ./bluethoot_mac.txt
