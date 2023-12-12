# ES ULP 3

[Kontextdiagramm, DFD und STD](/Documentation/Kontextdiagramm_DFD_STD.pdf)
[PAT](/Documentation/PAT.pdf)

###  Datenverzeichnis
 
`DATA state = " uint_8 "`  
`DATA errorId = " uint_8 "`  
`DATA partnerId = " uint_8 "`  
`DATA packageId = " uint_8 "`  
`DATA Lager = " uint_8[6] "`  
`DATA tempLager = " uint_8[6] "`  
`DATA ApNr = " uint_8 "`  

`DATA receive = " bool " * Kontrollfluss *`  
`DATA passOn = " bool " * Kontrollfluss *`  
`DATA create = " bool " * Kontrollfluss *`  
`DATA deliver = " bool " * Kontrollfluss *`  
`DATA poll = " bool " * Kontrollfluss *`  
`DATA await = " bool " * Kontrollfluss *`  
`DATA finishedSend = " bool " * Kontrollfluss *`
`DATA finishedStore = " bool " * Kontrollfluss *`
`DATA failure = " bool " * Kontrollfluss *`  
`DATA receivedSDU = " bool " * Kontrollfluss *`  
`DATA GPIO_neighbour_in = " bool " * Kontrollfluss *`  

### Minispezifikation

#### stateProcessing:
Setzt `state` auf 0. Dieser Code steht für den Status "processing", welcher dem Master über das MMCP-Protokoll zurückgemeldet wird.

#### stateAwait:
Setzt `state` auf 1. Dieser Code steht für den Status "awaiting", welcher dem Master über das MMCP-Protokoll zurückgemeldet wird.

#### stateReceived:
Setzt `state` auf 2. Dieser Code steht für den Status "received", welcher dem Master über das MMCP-Protokoll zurückgemeldet wird.

#### stateSent:
Setzt `state` auf 3. Dieser Code steht für den Status "sent", welcher dem Master über das MMCP-Protokoll zurückgemeldet wird.

#### stateFailure:
Setzt `state` auf 4. Dieser Code steht für den Status "failure", welcher dem Master über das MMCP-Protokoll zurückgemeldet wird.

#### handleStore:
Speichert Paket mit der Nummer `packageId` im `tempLager` Array an der ersten freien Stelle `i`, wo `tempLager[i] == 0` gilt.

#### handleSend:
Löscht Paket mit der Nummer `packageId` aus dem `tempLager` Array, setzt also an der Stelle `i` des Pakets `tempLager[i] = 0`.

#### updateLager:
Kopiert den `tempLager` Array zum `Lager` Array.

#### animateSend:
Senden eines Paketes auf der LED-Anzeige animieren und den Inhalt von `Lager` mit vorgeschriebenen Farben für jede verschiedene Paketnummer `packageId` darstellen.
#### animateReceive:
Empfangen eines Paketes auf der LED-Anzeige animieren und den Inhalt von `Lager` mit vorgeschriebenen Farben für jede verschiedene Paketnummer `packageId` darstellen.
#### animateCreate:
Erstellen eines Paketes auf der LED-Anzeige animieren und den Inhalt von `Lager` mit vorgeschriebenen Farben für jede verschiedene Paketnummer `packageId` darstellen.
#### animateDeliver:
Senden eines Paketes auf der LED-Anzeige animieren und den Inhalt von `Lager` mit vorgeschriebenen Farben für jede verschiedene Paketnummer `packageId` darstellen.

#### pulse:
Gibt eine steigende Flanke mit der Dauer 1ms auf dem zu `partnerId` korrespondierenden GPIO-Pin aus. Die Zuordnung von Pins und Nachbarn ist als Konstante im Code realisiert.

#### checkFailure:
Überprüft den empfangenen Befehl auf logische Fehler und Ausführbarkeit. 

Zuerst wird überprüft, ob `partnerId` ein bekannter Nachbar bzw. `0` ist. Ist dies nicht der Fall, so wird `errorId = 4` und `failure = TRUE` gesetzt. Die Liste der Nachbarn ist als Konstante im Code realisiert.

Ist `ApNr == 42`, ein Packet soll also erstellt oder empfangen werden, wird überprüft, ob der `Lager` Array noch einen freien Platz `Lager[i] == 0` hat. Ist dies nicht der Fall, so wird `errorId = 2` und `failure = TRUE` gesetzt. 
Zudem wird überprüft, ob `packageId` bereits im `Lager` Array vorhanden ist. Ist dies der Fall, so wird `errorId = 1` und `failure = TRUE` gesetzt.

Ist `ApNr == 43`, ein Packet soll gelöscht oder weitergeleitet werden, wird überprüft, ob der `packageId` im `Lager` Array exisitiert. Ist dies nicht der Fall, so wird `errorId = 3` und `failure = TRUE` gesetzt.

Zuletzt wird überprüft, ob `packageId` eine gültige Paketnummer (zwischen einschließlich 1 und 16) ist. Ist dies nicht der Fall, so wird `errorId = 5` und `failure = TRUE` gesetzt.

#### resetData
Setzt `packageId`, `partnerId`, `errorId` und `ApNr` auf `0`. Setzt `rceeive`, `passOn`, `create`, `deliver`, `poll`, `await`, `failure`, `finishedStore`, `finishedSend`, `receivedSDU`auf `FALSE`