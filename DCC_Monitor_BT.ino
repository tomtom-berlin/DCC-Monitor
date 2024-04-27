//-------------------------------------------------------------------------------------------------------
/*
    DCC-Monitor zur Ausgabe der DCC-Befehle auf dem seriellen Monitor
    =================================================================

    V 1.4, 17.12.2020 domapi

    NEU 17.12.2020
    - Anpassung der Pufferroutine bei Schaltartikel-Befehlen (Acc), da bei DCC++ teilweise kein "Signal aus"-Befehl gesendet wird. Die Folge war, dass Befehle verschluckt wurden.
    - Performance optimiert: 3 x break eingebaut in Puffer-Löschroutine. 

    NEU 12.12.2020
    - Der ESU-Lokprogrammer sendet offensichtlich zusätzliche DCC-Kommandos "Analogfunktionsgruppen"
    - diese wurden eingebaut, incl. Zähler für Spezialbefehle für Loks

    NEU 09.11.2020
    - Bei den Zubehördekodern wurde die Schaltrichtung/Spule umgedreht und die Textausgabe passt jetzt z.B. zur Anzeige im ESU Lokprogrammer Führerstand Weichenstellpult
      - rote Taste = A
      - grüne Taste = B
    - Bei Lokdekoder-Adressen von 112 - 127 wurden falsche DCC-Befehle ausgegeben, da diese vom Sketch fälschlicherweise als Programmierbefehle für das Programmiergleis interpretiert wurden.
    - Nun wird eine zusätzliche Routine der NRMA-Library aufgerufen, die prüft, ob der DCC-Monitor vom Programmiergleis angesprochen wurde.
    - In diesem Fall sendet die Zentrale diverse Befehlsketten, um den Dekoder bzw. Monitor in den Programmiermodus zu versetzen.
    - Die Programmierbefehle fürs Programmiergleis und die Befehle für kurze Lokadressen ähneln sich dummerweise stark ...

    Features:
    ---------
    - Ausgabe der Lok-Befehle
    - Ausgabe der Lok-CV-Kommandos
    - Ausgabe der Accessory-Befehle
    - Ausgabe der Accessory-CV-Kommandos
    - Ausgabe der CV-Befehle auf dem Programmiergleis
    - Eingaben über seriellen Monitor zur Steuerung der Anzeige
    - DCC-Paket-Speicher zum Verhindern der mehrfachen Ausgabe von Befehlen
      - einstellbar getrennt nach Loks, Accessories und CVs
      - Wurde ein gültiges Paket gefunden und im seriellen Monitor ausgegeben, wird es in einer Tabelle gespeichert
      - Vor jeder Paketausgabe wird geprüft, ob es in der Tabelle bereits enthalten ist, d.h. schon ausgegeben wurde
      - Die Tabelle umfasst für Loks 240 x 4 Bytes, ohne Präambel und ohne Check-Byte; für Zubehörbefehle nur 5 x 4 Bytes
        (Lok-Befehle werden sehr häufig und vor allem periodisch wiederholt, Zubehör mehrfach, aber nur bei Änderungen )
      - Die Tabelle wird sukzessive aufgefüllt, wenn sie voll ist, wird der erste Eintrag gelöscht und alle anderen Einträge nach vorne geschoben
        Das neue Paket wandert dann an die letzte Stelle
      - Die Tabellengröße sollte für ca. 70 Loks reichen
    - Über das Menü kann die Speicherung ein-/aus-geschaltet werden (separat für Acc, CVs, Loks)
    - Ausgabe der Loknamen in Klartext
    - Ausgabe einer Statistik über die detektierten DCC-Befehle

    Hardware:
    ---------
    - Schaltplan, siehe: https://www.stummiforum.de/viewtopic.php?t=165060&start=225#p1957001
    - DCC-Signal-Auswertung über 6N137 Optokoppler, Schutzdiode (1N4148) und Vorwiderstand (1k). Oder über Brückengleichrichter (siehe Link).
      Pin 6 Optokoppler mit 10 kOhm an 5V reicht aus. Pin 7 muss nicht an 5V angeschlossen sein, er kann unbelegt bleiben.
    - ACK-Signal über Optokoppler CNY17 und Transistor BC557 anschließen, erzeugt Stromimpuls an +/- des Brückengleichrichters.


    Arduino Nano:
    //                      +-----+
    //         +------------| USB |------------+
    //         |            +-----+            |
    //         | [ ]D13/SCK        MISO/D12[ ] |
    //         | [ ]3.3V           MOSI/D11[ ]~|
    //         | [ ]V.ref     ___    SS/D10[ ]~|
    //         | [ ]A0       / N \       D9[ ]~|
    //         | [ ]A1      /  A  \      D8[ ] |
    //         | [ ]A2      \  N  /      D7[ ] |
    //         | [ ]A3       \_0_/       D6[ ]~|
    //         | [ ]A4/SDA               D5[ ]~|
    // ACK-Pin | [ ]A5/SCL               D4[ ] |
    //         | [ ]A6              INT1/D3[ ]~|
    //         | [ ]A7              INT0/D2[ ] |   DCC-Eingang über Optokoppler Pin 6
    //         | [ ]5V                  GND[ ] |
    //         | [ ]RST                 RST[ ] |
    //         | [ ]GND   5V MOSI GND   TX1[ ] |
    //         | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |
    //         |          [ ] [ ] [ ]          |
    //         |          MISO SCK RST         |
    //         | NANO-V3                       |
    //         +-------------------------------+
*/
//-------------------------------------------------------------------------------------------------------
#include <NmraDcc.h>
#include "BluetoothSerial.h"

const char* app_version = "1.4";
const char* device_name = "ESP32-DCC-Monitor";

// workaround for compiler error
#undef min
inline int min(uint32_t a, unsigned int b) {
  return ((a) < (b) ? (a) : (b));
}

// Check ob Bluetooth Serial verfuegbar
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED) && defined(CONFIG_BT_SPP_ENABLED)
BluetoothSerial SerialBT;
#endif

NmraDcc Dcc;
//#define debug

//-------------------------------------------------------------------------------------------------------
// Mit diesen Variablen kann man einstellen, was auf dem seriellen Monitor ausgegeben wird
// Die Einstellungen können während des laufenden Betriebs des DCC-Monitors später im seriellen Monitor geändert werden
//-------------------------------------------------------------------------------------------------------
byte Anzeige_Loks = 1;  // Lok-Befehle für Lok-Dekoder und funktionsdekoder
byte Anzeige_Acc = 1;   // Zubehör-Befehle für Weichendekoder
byte Anzeige_CV = 1;    // Ausgabe von CV-Aktionen (Lesen und Schreiben etc.)

byte puffern_Lok = 1;  // Schaltet die Speicherung und Prüfung bereits empfangener DCC-Pakete ein/aus
byte puffern_Acc = 1;
byte puffern_CV = 1;
//-------------------------------------------------------------------------------------------------------

const byte DccInterruptPin = 27;
const byte DccAckPin = 26;  // Arduino-Pin zur Erzeugung eines ACK-Signals
const char* comp_date = __DATE__ ", " __TIME__;

int blinker_aus = 10000;  // toggelt die LED an Pin LED_BUILTIN, wenn ein DCC-Paket gefunden wurde --> zeigt DCC-Signal an
int blinker_an = 5;

const byte bufferSizeAcc = 10;  // Schaltartikelbefehle werden nicht andauernd wiederholt; hier reichen ein paar Pufferplätze aus
const byte bufferSizeLok = 240;
byte Acc_counter = 0;  // läuft von 1 - bufferSizeAcc
byte Paket_bekannt_A = 0;
byte Lok_counter = 0;  // läuft von 1 - bufferSizeLok
byte Paket_bekannt_L = 0;

// Strukturen zur Speicherung der bereits empfangenen Befehle
typedef struct
{
  int WADR;
  byte DIR;
  byte POW;
} ACC_Befehl;

typedef struct
{
  int ADR;
  byte ORDER;
  byte FUNC;
} Lok_Befehl;


ACC_Befehl Acc_received[bufferSizeAcc];
Lok_Befehl Lok_received[bufferSizeLok];


byte pktByteCount = 0;
unsigned int decoderAddress;
unsigned int decoderAddress_alt = 0;
unsigned int Weichenadresse;
byte Ausgang;
byte Direction;
byte Power;
byte Befehl;
byte Funktion;

byte Befehls_Byte;
byte decoderType;  //0=Lok, 1=Zubehör/Accessory

byte command;
int CV_address;
byte CV_value;

byte command_alt = 0;
int CV_address_alt = 0;
byte CV_value_alt = 0;

byte speed;

byte checksum = 0;

// Zähler für die Statistik
unsigned long start_time = 0;
unsigned long z_bytes = 0;
unsigned long z_invalid = 0;
unsigned long z_idle = 0;
unsigned long z_lok_speed = 0;
unsigned long z_lok_F0 = 0;
unsigned long z_lok_F5 = 0;
unsigned long z_lok_F9 = 0;
unsigned long z_lok_F13 = 0;
unsigned long z_lok_F21 = 0;
unsigned long z_lok_F29 = 0;
unsigned long z_acc = 0;
unsigned long z_dec_reset = 0;
unsigned long z_acc_cv = 0;
unsigned long z_lok_cv = 0;
unsigned long z_prg_CV = 0;
unsigned long z_ack = 0;
unsigned long z_spezial = 0;
bool Service_Mode;

//-------------------------------------------------------------------------------------------------------
void setup()
//-------------------------------------------------------------------------------------------------------
{
  pinMode(LED_BUILTIN, OUTPUT);  // eingebaute LED
  Serial.begin(115200);
  SerialBT.begin(device_name);
  while (!SerialBT) {
    // Warte, bis die serielle Schnittstelle verbunden ist.
  }

  pinMode(DccAckPin, OUTPUT);  // Configure the DCC CV Programing ACK pin for an output
  Serial.println(F("Bluetooth aktiviert."));
  Serial.print(F("Das Gerät "));
  Serial.print(F(device_name));
  Serial.println(F(" ist zum Pairing bereit."));
  SerialBT.print(F("NMRA DCC Monitor V "));
  SerialBT.println(F(app_version));
  SerialBT.print(F("Sketch-Upload am: "));
  SerialBT.println(comp_date);
  SerialBT.println(F("? = Zeige Tastaturbefehle für den seriellen Monitor"));

// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
#ifdef digitalPinToInterrupt
  Dcc.pin(DccInterruptPin, 0);
#else
  Dcc.pin(0, DccInterruptPin, 1);
#endif
  Dcc.init(MAN_ID_DIY, 12, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);  // Call the main DCC Init function to enable the DCC Receiver

  start_time = millis();
}

//-------------------------------------------------------------------------------------------------------
void loop()
//-------------------------------------------------------------------------------------------------------
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if (SerialBT.available()) {
    // Tastatur-Befehle via seriellen Monitor des Arduinos:

    // 1 = Anzeige Loks ein/aus
    // 2 = Anzeige Zubehör ein/aus
    // 3 = Anzeige CV-Befehle ein/aus
    // 4 = Nur neue Lok-Pakete ein/aus
    // 5 = Nur neue Zubehör-Pakete ein/aus
    // 6 = Nur neue CV-Befehle ein/aus
    // 7 = Statistik
    // ? = Befehle anzeigen

    switch (SerialBT.read()) {
      case 49:  // 1
        SerialBT.print(F("1 Anzeige Loks ein/aus = "));
        Anzeige_Loks = !Anzeige_Loks;
        if (Anzeige_Loks) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 50:  //2
        SerialBT.print(F("2 Anzeige Zubehör ein/aus = "));
        Anzeige_Acc = !Anzeige_Acc;
        if (Anzeige_Acc) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 51:  // 3
        SerialBT.print(F("3 Anzeige CV-Befehle ein/aus = "));
        Anzeige_CV = !Anzeige_CV;
        if (Anzeige_CV) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 52:  // 4
        SerialBT.print(F("4 Nur neue Lok-Pakete anzeigen ein/aus = "));
        puffern_Lok = !puffern_Lok;
        if (puffern_Lok) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 53:  // 5
        SerialBT.print(F("5 Nur neue Zubehör-Pakete anzeigen ein/aus = "));
        puffern_Acc = !puffern_Acc;
        if (puffern_Acc) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 54:  // 6
        SerialBT.print(F("6 Nur neue CV-Befehle anzeigen ein/aus = "));
        puffern_CV = !puffern_CV;
        if (puffern_CV) SerialBT.println("ein");
        else SerialBT.println("aus");
        break;
      case 55:  // 7
        SerialBT.println();
        SerialBT.println(F("S t a t i s t i k"));
        SerialBT.println(F("-----------------"));
        SerialBT.print(F("Zeitraum [sec]         :"));
        print_Zahl_rechts_ln((millis() - start_time) / 1000);

        SerialBT.print(F("Anzahl empfangene Bytes:"));
        print_Zahl_rechts_ln(z_bytes);
        SerialBT.print(F("Gültige Kommandos      :"));
        print_Zahl_rechts_ln(z_invalid + z_idle + z_lok_speed + z_lok_F0 + z_lok_F5 + z_lok_F9 + z_lok_F13 + z_lok_F21 + z_lok_F29 + z_acc + z_dec_reset + z_acc_cv + z_lok_cv + z_prg_CV);
        SerialBT.print(F("Ungültige Kommandos    :"));
        print_Zahl_rechts_ln(z_invalid);
        SerialBT.print(F("Idle-Pakete            :"));
        print_Zahl_rechts_ln(z_idle);

        SerialBT.print(F("Geschwindigkeitsbefehle:"));
        print_Zahl_rechts_ln(z_lok_speed);
        SerialBT.print(F("F0 - F4 Funktionen     :"));
        print_Zahl_rechts_ln(z_lok_F0);
        SerialBT.print(F("F5 - F8 Funktionen     :"));
        print_Zahl_rechts_ln(z_lok_F5);
        SerialBT.print(F("F9 - F12 Funktionen    :"));
        print_Zahl_rechts_ln(z_lok_F9);
        SerialBT.print(F("F13 - F20 Funktionen   :"));
        print_Zahl_rechts_ln(z_lok_F13);
        SerialBT.print(F("F21 - F28 Funktionen   :"));
        print_Zahl_rechts_ln(z_lok_F21);
        SerialBT.print(F("F29 - F36 Funktionen   :"));
        print_Zahl_rechts_ln(z_lok_F29);

        SerialBT.print(F("Spezialbefehle Lok     :"));
        print_Zahl_rechts_ln(z_spezial);

        SerialBT.print(F("Zubehör-Befehle        :"));
        print_Zahl_rechts_ln(z_acc);

        SerialBT.print(F("Dekoder-Reset-Befehle  :"));
        print_Zahl_rechts_ln(z_dec_reset);
        SerialBT.print(F("Zubehör-CV-Befehle     :"));
        print_Zahl_rechts_ln(z_acc_cv);
        SerialBT.print(F("Lok-CV-Befehle         :"));
        print_Zahl_rechts_ln(z_lok_cv);
        SerialBT.print(F("Programmiergleisbefehle:"));
        print_Zahl_rechts_ln(z_prg_CV);
        SerialBT.print(F("Acknowledgments        :"));
        print_Zahl_rechts_ln(z_ack);

        SerialBT.print(F("Counter Lok            :"));
        print_Zahl_rechts_ln(Lok_counter);
        SerialBT.print(F("Counter Acc            :"));
        print_Zahl_rechts_ln(Acc_counter);
        break;
      case 63:  // ?
        SerialBT.println();
        SerialBT.println(F("Tastaturbefehle für den seriellen Monitor:"));
        SerialBT.println();
        SerialBT.print(F("1 = Anzeige Loks ein/aus                     "));
        if (Anzeige_Loks) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.print(F("2 = Anzeige Zubehör ein/aus                  "));
        if (Anzeige_Acc) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.print(F("3 = Anzeige CV-Befehle ein/aus               "));
        if (Anzeige_CV) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.print(F("4 = Nur neue Lok-Pakete anzeigen ein/aus     "));
        if (puffern_Lok) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.print(F("5 = Nur neue Zubehör-Pakete anzeigen ein/aus "));
        if (puffern_Acc) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.print(F("6 = Nur neue CV-Befehle ein/aus              "));
        if (puffern_CV) SerialBT.println("ein");
        else SerialBT.println("aus");
        SerialBT.println(F("7 = Statistik anzeigen"));
        SerialBT.println(F("? = Befehle anzeigen"));
        break;
    }
    SerialBT.println(" ");
  }
  // LED ausschalten
  blinker_aus--;
  if (blinker_aus == 0) {
    digitalWrite(LED_BUILTIN, 0);  // wenn kein DCC-Signal mehr kommt, die LED ausschalten
    blinker_aus = 10000;
  }
}


//-------------------------------------------------------------------------------------------------------
// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
void notifyCVAck()
//-------------------------------------------------------------------------------------------------------
{
  digitalWrite(DccAckPin, HIGH);
  delay(6);
  digitalWrite(DccAckPin, LOW);
  z_ack++;
}


//-------------------------------------------------------------------------------------------------------
// Wird aufgerufen, wenn ein Progammierbefehl von der Zentrale gesendet wurde
// Brauchen wir, um nicht mit den Lok-Adressen 112 - 127 zu kollidieren ;-)
void notifyServiceMode(bool Prg_gleis_Mode)
//-------------------------------------------------------------------------------------------------------
{
  SerialBT.print("Service-Mode/Programmiergleismodus: ");
  if (Service_Mode == 0) {
    SerialBT.println("Ein");
  } else {
    SerialBT.println("Aus");
  }
  Service_Mode = Prg_gleis_Mode;
}



//-------------------------------------------------------------------------------------------------------
void notifyDccMsg(DCC_MSG* Msg)
//-------------------------------------------------------------------------------------------------------
{
  // LED einschalten
  blinker_an--;
  if (blinker_an == 0) {
    digitalWrite(LED_BUILTIN, 1);
    blinker_an = 5;
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  // Alle gefundenen Bytes XOR-verknüpfen; muss 0 ergeben, dann wurde ein gültiger Befehl gefunden!
  //-------------------------------------------------------------------------------------------------------------------------------

  pktByteCount = Msg->Size;  // Anzahl gefundene Bytes ohne Präambel aber incl. Prüfbyte !!!
  checksum = 0;              // Wir starten mit 0
  z_bytes = z_bytes + pktByteCount;

  for (byte n = 0; n < pktByteCount; n++) {
    checksum ^= Msg->Data[n];
  }
  if (checksum) {
    z_invalid++;
    return;  // Ungültige Checksumme --> nix tun !
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  // Start Dekodierung
  //-------------------------------------------------------------------------------------------------------------------------------

  //-------------------------------------------------------------------------------------------------------------------------------
  // Idle Kommando
  //-------------------------------------------------------------------------------------------------------------------------------
  if (Msg->Data[0] == 0b11111111) {
    z_idle++;
    return;  // Idle packet
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  // Reset Befehl zur Einleitung der CV-Programmierung
  //-------------------------------------------------------------------------------------------------------------------------------
  if (Msg->Data[0] == 0) {
    z_dec_reset++;

    command = Msg->Data[0];
    CV_address = Msg->Data[1];
    CV_value = Msg->Data[2];

    // Nur verarbeiten, wenn neuer Befehl!
    if (!(((CV_value == CV_value_alt) && (CV_address == CV_address_alt) && (command == command_alt)) && puffern_CV)) {
      SerialBT.print(F("Prg   Dekoder-Reset-Befehl"));
      print_spaces(43);
      printPacket(Msg);

      command_alt = command;
      CV_address_alt = CV_address;
      CV_value_alt = CV_value;
    }
    return;
  }
  //-------------------------------------------------------------------------------------------------------------------------------
  // Programmiermodus auf dem Programmiergleis ohne Adresse !
  //-------------------------------------------------------------------------------------------------------------------------------

  // Achtung:
  // Die Programmierbefehle ohne Adresse kollidieren mit den Lok-Adressen 112 - 127 !!!!!!!!!!!!!!!!!!
  // Der Dekoder muss erst erkennen, dass er im Programmiermodus angesprochen wird.
  // Hierfür werden verschiedene Sequenzen von Befehlen von der Zenrale gesendet.
  // Ob man im Prg.modus auf dem Prg.gleis ist, kann mit der NRMA-Routine notifyServiceMode() geprüft werden.


  if (Service_Mode && (Msg->Data[0] > 111) && (Msg->Data[0] < 128)) {
    /*  Service Mode.Prog: [preamble] 0 [0111CCVV] 0 [VVVVVVVV] 0 [DDDDDDDD] 0 [EEEEEEEE] 1
                                                 CC = Command
                                                   VV VVVVVVVV = 10 bit CV Number
                                                                       DDDDDDDD = New Value (8 bit)
                                                                                    EEEEEEEE = Checksum
            Lesen/Schreiben auf dem Prg.gleis geht ohne Dekoderadresse !

            Schreiben: z.B. CV 6, Wert 20 (--> 4 Bytes ohne Präambel und Trennbits!)
            Byte 0   Byte 1   Byte 2   Byte 3
            01111100 -----101 ---10100 10000001

            0111CCVV VVVVVVVV DDDDDDDD EEEEEEEE
                11 = Schreiben
                01 = Lesen/Überprüfen
                10 = 10 Bit Manipulation
                  V = CV - 1 --> 5 = CV 5 + 1 = CV6 !
                                       D = 20                */
    z_prg_CV++;

    // Nur verarbeiten, wenn neuer Befehl!
    command = Msg->Data[0] & 0b00001100;                                  // 2 Bits enthalten den Schreib-/ Verify-Befehl etc.
    CV_address = ((Msg->Data[0] & 0b00000011) * 256) + Msg->Data[1] + 1;  // Nummer der CV
    CV_value = Msg->Data[2];                                              // CV-Wert für das Schreiben

    if (!(((CV_value == CV_value_alt) && (CV_address == CV_address_alt) && (command == command_alt)) && puffern_CV)) {
      decoderType = 255;  // vorsichtshalber mal auf einen dämlichen Wert setzen, damit weiter unten nix passiert

      SerialBT.print("Prg   CV");

      CV_address = ((Msg->Data[0] & 0b00000011) * 256) + Msg->Data[1] + 1;
      SerialBT.print(CV_address);
      if (CV_address < 1000) SerialBT.print(" ");
      if (CV_address < 100) SerialBT.print(" ");
      if (CV_address < 10) SerialBT.print(" ");
      SerialBT.print(" ");

      switch (Msg->Data[0] & 0b00001100) {
        /*  Die für den Befehlstyp (xxxx-KKxx) im ersten Befehlsbyte festgelegten Werte sind:
                      KK = 00 – reserviert
                      KK = 01 – Byte Überprüfen
                      KK = 11 – Byte Schreiben
                      KK = 10 – Bit Manipulation*/

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 0b00000100:  // Verify Byte
          SerialBT.print(F("Lese CV"));
          print_spaces(49);
          break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 0b00001100:  // Write Byte

          SerialBT.print(F("Schreibe CV ="));
          if (Msg->Data[2] < 100) SerialBT.print(" ");
          if (Msg->Data[2] < 10) SerialBT.print(" ");
          SerialBT.print(" ");
          SerialBT.print(Msg->Data[2]);
          print_spaces(39);
          break;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 0b00001000:  // Bit Write
          // 0111-10VV VVVV-VVVV 111K-DBBB EEEE-EEEE --> im Programmiermodus für Zugriffe auf einzelne Bits
          // K = 1 – Bit Schreiben
          // K = 0 – Bit Überprüfen
          if (Msg->Data[2] & 0b00010000) {
            SerialBT.print("Schreibe Bit #");
            SerialBT.print(Msg->Data[2] & 0b00000111);
            SerialBT.print(" = ");
            SerialBT.print((Msg->Data[2] & 0b00001000) >> 3);
            print_spaces(36);
          } else {
            SerialBT.print("Lese    Bit #");
            SerialBT.print(Msg->Data[2] & 0b00000111);
            print_spaces(42);
          }
          break;
      }
      printPacket(Msg);

      command_alt = command;
      CV_address_alt = CV_address;
      CV_value_alt = CV_value;

      return;
    }
  } else {
    //-------------------------------------------------------------------------------------------------------------------------------
    // 0xxx-xxxx -->  bit7=0 -> Lok Dekoder kurze Adresse
    //-------------------------------------------------------------------------------------------------------------------------------
    if (!bitRead(Msg->Data[0], 7)) {
      decoderType = 0;                // Lok
      decoderAddress = Msg->Data[0];  // kurze Adresse
      Befehls_Byte = Msg->Data[1];

      // Aufteilung der gefundenen Bytes auf Befehle (Was soll der Dekoder tun?) und Funktionen (Wie soll er es tun?)

      if ((Befehls_Byte & 0b11100000) == 0b10000000) {
        Befehl = 0b00000100;                   // 100 = F0 - F4
        Funktion = Befehls_Byte & 0b00011111;  // x-xxxx = F0 F4 - F1
        z_lok_F0++;
      }

      else if ((Befehls_Byte & 0b11110000) == 0b10110000) {
        Befehl = 0b00001011;                   // 1011 = F5 - F8
        Funktion = Befehls_Byte & 0b00001111;  // xxxx = F8 - F5
        z_lok_F5++;
      }

      else if ((Befehls_Byte & 0b11110000) == 0b10100000) {
        Befehl = Befehls_Byte >> 4;            // 1010 = F9 - F12
        Funktion = Befehls_Byte & 0b00001111;  // xxxx = F12 - F9
        z_lok_F9++;
      }

      else if (Befehls_Byte == 0b11011110) {
        Befehl = Befehls_Byte;    // 1101-1110 = F13 - F20
        Funktion = Msg->Data[2];  // xxxx-xxxx = F20 - F13
        z_lok_F13++;
      } else if (Befehls_Byte == 0b11011111) {
        Befehl = Befehls_Byte;    // 1101-1111 = F21 - F28
        Funktion = Msg->Data[2];  // xxxx-xxxx = F28 - F21
        z_lok_F21++;
      } else if (Befehls_Byte == 0b11011000) {
        Befehl = Befehls_Byte;    // 1101-1000 = F29 - F36
        Funktion = Msg->Data[2];  // xxxx-xxxx = F36 - F29
        z_lok_F29++;
      } else if ((Befehls_Byte & 0b11000000) == 64 && pktByteCount == 3) {
        Befehl = 0;               // 01RSSSSS
        Funktion = Befehls_Byte;  // Speed 28 Stufen
        z_lok_speed++;

      } else if (Befehls_Byte == 0b00111111 && pktByteCount == 4) {
        Befehl = Befehls_Byte;    // 00111111
        Funktion = Msg->Data[2];  // RSSSSSSS Speed 127 Stufen
        z_lok_speed++;
      }

      // NEU 12.12.2020
      else if (Befehls_Byte == 0b00111101)  // 0011-1101 scheint ein Spezialkommando des ESU Lokprogrammers zu sein. Ws. eine Kennung für den Dekoder, dass er von einem Lokprogrammer angesprochen wird
      {
        Befehl = Befehls_Byte;
        Funktion = Msg->Data[2];  // Erweiterter Betriebsbefehl Analogfunktionsgruppe
        z_spezial++;
      }
      // NEU 12.12.2020

    } else {
      //-------------------------------------------------------------------------------------------------------------------------------
      // 11xx-xxxx -->  bit7 = 1 AND bit6 = 1 -> Lok Dekoder lange Adresse
      //-------------------------------------------------------------------------------------------------------------------------------
      if (bitRead(Msg->Data[0], 6)) {
        // 11xx-xxxx
        decoderAddress = 256 * (Msg->Data[0] & 0b00111111) + Msg->Data[1];
        Befehls_Byte = Msg->Data[2];
        decoderType = 0;

        // Aufteilung der gefundenen Bytes auf Befehle und Funktionen

        if ((Befehls_Byte & 0b11100000) == 0b10000000) {
          Befehl = 0b00000100;                   // 100 = F0 - F4
          Funktion = Befehls_Byte & 0b00011111;  // x-xxxx = F0 F4 - F1
          z_lok_F0++;
        }

        else if ((Befehls_Byte & 0b11110000) == 0b10110000) {
          Befehl = 0b00001011;                   // 1011 = F5 - F8
          Funktion = Befehls_Byte & 0b00001111;  // xxxx = F8 - F5
          z_lok_F5++;
        }

        else if ((Befehls_Byte & 0b11110000) == 0b10100000) {
          Befehl = 0b00001010;                   // 1010 = F9 - F12
          Funktion = Befehls_Byte & 0b00001111;  // xxxx = F12 - F9
          z_lok_F9++;
        }

        else if (Befehls_Byte == 0b11011110) {
          Befehl = Befehls_Byte;    // 1101-1110 = F13 - F20
          Funktion = Msg->Data[3];  // xxxx-xxxx = F20 - F13
          z_lok_F13++;
        } else if (Befehls_Byte == 0b11011111) {
          Befehl = Befehls_Byte;    // 1101-1111 = F21 - F28
          Funktion = Msg->Data[3];  // xxxx-xxxx = F28 - F21
          z_lok_F21++;
        } else if (Befehls_Byte == 0b11011000) {
          Befehl = Befehls_Byte;    // 1101-1000 = F29 - F36
          Funktion = Msg->Data[2];  // xxxx-xxxx = F36 - F29
          z_lok_F29++;
        } else if ((Befehls_Byte & 0b11000000) == 64 && pktByteCount == 4) {
          Befehl = 0;  // 01RSSSSS
          z_lok_speed++;
          Funktion = Befehls_Byte;  // Speed 28 Stufen
        } else if (Befehls_Byte == 0b00111111 && pktByteCount == 5) {
          Befehl = Befehls_Byte;    // 00111111
          Funktion = Msg->Data[3];  // RSSSSSSS Speed 127 Stufen
          z_lok_speed++;
        }
        // NEU 12.12.2020
        else if (Befehls_Byte == 0b00111101) {
          Befehl = Befehls_Byte;    //
          Funktion = Msg->Data[3];  // Analogfunktionsgruppe
          z_spezial++;
        }
        // NEU 12.12.2020
      }
      //-------------------------------------------------------------------------------------------------------------------------------
      else  //bit7=1 AND bit6=0 -> Accessory Decoder
      {
        // 10xx-xxxx
        //-------------------------------------------------------------------------------------------------------------------------------

        decoderAddress = Msg->Data[0] & 0b00111111;
        Befehls_Byte = Msg->Data[1];
        decoderType = 1;
      }
    }
  }
  //-------------------------------------------------------------------------------------------------------------------------------
  if (decoderType == 1)  // Accessory Basic
  {
    if (Anzeige_Acc && (Msg->Size != 6)) {
      z_acc++;

      if ((Befehls_Byte & 0b10000000) && (Msg->Size == 3))  // Steuerbefehl für Zubehör Dekoder (Basic Accessory)
      {
        // Dekoder haben eine Dekoderadresse (die wird zum Programmieren der CVs verwendet) und umfassen i.d.R. 4 Ausgänge zum Ansteuern vn Schaltartikel mit wiederum jeweils 2 Richtungen/Zuständen.
        // Die Zentrale sendet für Magnetartikel (accessories) immer ein Paket bestehend aus Dekoder-Adresse (= Signal), Ausgang (0 - 3), Richtung (rot/grün bzw. A/B) und Power (ein/aus).
        // Zunächst wird eine Richtung eingeschaltet; dazu wird das DCC-Telegramm ggf. mehrfach wiederholt
        // Bei der ECOS wird nach einer einstellbaren Zeit pro "Magnetartikel" ein Ausschaltbefehl (auch mehrfach) hinterhergeschickt

        /*  Für Zubehör-/Weichendekoder sendet die DCC-Zentrale 3 Bytes:
                    Byte 1                      Byte 2                        Byte 3
                                                   __ __ __
                    1  0 A7 A6 A5 A4 A3 A2   :   1 AA A9 A8  P A1 A0  R   :   C7 C6 C5 C4 C3 C2 C1 C0

                    - AA..A0 sind die 11 Bit Adresse eines Zubehördekoders ("Addr") (entspricht quasi der Dekoderadresse und dem Ausgang)
                    - P = Power (0 = off, 1 = on) ("OutputPower") also bei Doppelspulenantrieben z.B. "ein" oder "aus"
                    - R = Richtung/Direction, z.B. Abzweig/Gerade oder rot/grün (in welche Richtung sich z.B. der Servo bewegt, 0 = rot, 1 = grün)
                    - C7..C0 = Checkbyte (Byte_3 = Byte_1 XOR Byte_2)

                    - Das 1. Byte beginnt immer mit "10" --> 10XX XXXX (Bit#7 = 1, Bit#6 = 0)
                    - Das 2. Byte beginnt immer mit "1"  --> 1XXX XXXX (Bit#7 = 1)
                    - Das 3. Byte dient der überprüfung der gesendeten Informationen: Byte_3 = Byte_1 XOR Byte_2

                    - Die 11 Bit Adresse des zu steuernden Spulenpaares entsteht aus den 11 Adress-Bits (AA ... A0). Dabei ist zu
                      beachten, dass die Bits AA, A9, A8 invertiert im ursprünglichen DCC-Paket abgebildet sind.
                */

        decoderAddress = (((~Befehls_Byte) & 0b01110000) << 2) + decoderAddress;  // das ist die DCC-Adresse des Dekoders
        Ausgang = (Befehls_Byte & 0b00000110) >> 1;                               // entspricht 0 - 3 bzw. Ausgang 1 - 4 eines Dekoders

        Weichenadresse = (decoderAddress - 1) * 4 + Ausgang + 1;  // das ist die DCC-Adresse des Schaltartikels
        Direction = (bitRead(Befehls_Byte, 0));                   // Richtung A/B
        Power = (bitRead(Befehls_Byte, 3));                       // Power on/off

        if (puffern_Acc) {
          //-------------------------------------------------------------------------------------------------------------------------------
          // alle Werte im Puffer ausgeben
#ifdef debug
          SerialBT.println(F("1. Befehl erkannt, Stand Puffer vor der Verarbeitung:"));

          for (byte j = 0; j < bufferSizeAcc; j++) {
            SerialBT.print(j);
            SerialBT.print(" = ");
            SerialBT.print(Acc_received[j].WADR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Acc_received[j].DIR, HEX);
            SerialBT.print(" | ");
            SerialBT.println(Acc_received[j].POW, HEX);
          }
#endif
          // Schauen im Array, ob die gefundenen Bytes schon einmal ausgegeben wurden
          if (Acc_counter > 0) {
            for (byte j = 0; j < Acc_counter; j++) {
              Paket_bekannt_A = 0;
#ifdef debug
              SerialBT.println(F("2. Prüfen, ob neu oder bekannt:"));
              SerialBT.print(j);
              SerialBT.print(" = ");
              SerialBT.print(Acc_received[j].WADR, HEX);
              SerialBT.print(" | ");
              SerialBT.print(Weichenadresse, HEX);
              SerialBT.print(" | ");
              SerialBT.print(Acc_received[j].DIR, HEX);
              SerialBT.print(" | ");
              SerialBT.print(Direction, HEX);
              SerialBT.print(" | ");
              SerialBT.print(Acc_received[j].POW, HEX);
              SerialBT.print(" | ");
              SerialBT.print(Power, HEX);
              SerialBT.println(" | ");
#endif
              if (Acc_received[j].WADR == Weichenadresse) {
                Paket_bekannt_A++;
              }
              if (Acc_received[j].DIR == Direction) {
                Paket_bekannt_A++;
              }
              if (Acc_received[j].POW == Power) {
                Paket_bekannt_A++;
              }

              if (Paket_bekannt_A == 3) {
#ifdef debug
                SerialBT.print(F("--> Paket bereits bekannt"));
                SerialBT.println(" ");
#endif
                return;  // nix machen, keine Ausgabe im seriellen Monitor: war lediglich eine Wiederholung !
              }
            }
          }
#ifdef debug
          SerialBT.println();
          SerialBT.println(F("Neuer Befehl erkannt !"));
#endif
          if (Acc_counter > 0) {
            // Pufferzeilen mit der gleichen Adresse und Richtung rauslöschen
            for (byte j = 0; j < Acc_counter; j++) {
              //                            if (Acc_received [j].WADR == Weichenadresse && Acc_received [j].DIR == Direction)

              if (Acc_received[j].WADR == Weichenadresse)  // bei DCC++ wird kein "off-Befehl" gesendet, hier alte Befehle mit gleicher Weichenadresse löschen
              {
                // Befehl mit der gleichen Adresse aus der Puffertabelle löschen!
                // dazu beginnend mit der Zeile j alle weiter hinten vorziehen, bis Pufferende oder mindestens bis Acc_counter
                for (byte k = j; k < min(Acc_counter, bufferSizeAcc - 1); k++) {
                  Acc_received[k].WADR = Acc_received[k + 1].WADR;
                  Acc_received[k].DIR = Acc_received[k + 1].DIR;
                  Acc_received[k].POW = Acc_received[k + 1].POW;
                }

                //                           }
                Acc_counter--;  // ein Eintrag wurde gelöscht
#ifdef debug
                SerialBT.println();
                SerialBT.print(F("3. 1 Pufferzeile gelöscht: "));
                SerialBT.println(Weichenadresse, HEX);
#endif
                // es kann eigentlich nur eine Zeile im Puffer stehen mit gleicher Adresse und Richtung, wenn die gelöscht ist, kann man aufhören
                break;
              }
            }
          }
#ifdef debug
          // alle Werte im Puffer ausgeben
          SerialBT.println();
          SerialBT.println(F("4. Stand Puffer nach Löschen:"));

          for (byte j = 0; j < bufferSizeAcc; j++) {
            SerialBT.print(j);
            SerialBT.print(" = ");
            SerialBT.print(Acc_received[j].WADR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Acc_received[j].DIR, HEX);
            SerialBT.print(" | ");
            SerialBT.println(Acc_received[j].POW, HEX);
          }
#endif
          // eine Zeile im Acc_received mit neuem Paket befüllen
          if (Acc_counter < bufferSizeAcc) {
            Acc_received[Acc_counter].WADR = Weichenadresse;
            Acc_received[Acc_counter].DIR = Direction;
            Acc_received[Acc_counter].POW = Power;
#ifdef debug
            SerialBT.println();
            SerialBT.println(F("5. Fülle Puffer:"));
            SerialBT.print(Acc_counter);
            SerialBT.print(" = ");
            SerialBT.print(Acc_received[Acc_counter].WADR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Acc_received[Acc_counter].DIR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Acc_received[Acc_counter].POW, HEX);
            SerialBT.println(F(" --> Neuer Eintrag"));
#endif
            Acc_counter++;
          } else {
            // ersten ältesten Wert löschen und alle anderen Wertepaare nach links rücken
            for (byte j = 0; j < bufferSizeAcc - 1; j++) {
              Acc_received[j].WADR = Acc_received[j + 1].WADR;
              Acc_received[j].DIR = Acc_received[j + 1].DIR;
              Acc_received[j].POW = Acc_received[j + 1].POW;
            }
            // den letzten Eintrag nun mit dem neuen Paket befüllen

            Acc_received[bufferSizeAcc - 1].WADR = Weichenadresse;
            Acc_received[bufferSizeAcc - 1].DIR = Direction;
            Acc_received[bufferSizeAcc - 1].POW = Power;

            Acc_counter = bufferSizeAcc;
          }
#ifdef debug
          // alle Werte im Puffer ausgeben
          SerialBT.println();
          SerialBT.println(F("6. Stand Puffer nach der Verarbeitung:"));
          for (byte j = 0; j < bufferSizeAcc; j++) {
            SerialBT.print(j);
            SerialBT.print(" = ");
            SerialBT.print(Acc_received[j].WADR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Acc_received[j].DIR, HEX);
            SerialBT.print(" | ");
            SerialBT.println(Acc_received[j].POW, HEX);
          }
#endif
        }
        //-------------------------------------------------------------------------------------------------------------------------------
        // Zubehör-Daten ausgeben

        SerialBT.print(F("DCC-Adresse "));
        if (Weichenadresse < 1000) SerialBT.print(" ");
        if (Weichenadresse < 100) SerialBT.print(" ");
        if (Weichenadresse < 10) SerialBT.print(" ");
        SerialBT.print(Weichenadresse);
        SerialBT.print(" (");
        if (decoderAddress < 100) SerialBT.print(" ");
        if (decoderAddress < 10) SerialBT.print(" ");
        SerialBT.print(decoderAddress);
        SerialBT.print(" : ");
        SerialBT.print(Ausgang + 1);
        SerialBT.print(")");
        if (!bitRead(Befehls_Byte, 0)) SerialBT.print(" A");
        else SerialBT.print(" B");
        if (bitRead(Befehls_Byte, 3)) SerialBT.print(" On ");
        else SerialBT.print(" Off");
        print_spaces(37);
      } else  // Accessory Extended NMRA --> noch nicht getestet !!!
      {
        SerialBT.print("Acc Ext ");
        decoderAddress = (decoderAddress << 5) + ((Befehls_Byte & 0b01110000) >> 2) + ((Befehls_Byte & 0b00000110) >> 1);
        SerialBT.print(decoderAddress);
        SerialBT.print(" Asp ");
        SerialBT.print(Msg->Data[2], BIN);
      }
      printPacket(Msg);
    }

    //----------------------------------------------------------------------------------------------------------------------------------------
    // CV-Befehle für Schaltdekoder POM

    if (Anzeige_CV && (Msg->Size == 6))  // 6 Bytes --> d.h. CV-Befehle für Schaltdekoder !
    {
      z_acc_cv++;

      /*  ECOS: POM-Schaltartikel POM-Adresse 12, CV 6, Wert 20 Schreiben:  (6 Bytes ohne Präambel und Trennbits!)
                Byte 0   Byte 1   Byte 2   Byte 3   Byte 4   Byte 5
                10001100 11110000 11101100 -----101 ---10100 10000001
                          ___                                                  3 MSB der Adresse sind invertiert !
                10AAAAAA 1AAACDDD 1110CCVV VVVVVVVV DDDDDDDD EEEEEEEE
                    11 = Adresse 12
                                      11 = Schreiben
                                      10 = Lesen
                                        V = CV + 1 --> 5 + 1 = 6
                                                       20
                Lesen:
                10001100 11110000 11100100 -----101 -------0 10011101

                Bas.Op.Mode.Prog [preamble]0[10AAAAAA]0[1AAACDDD]0[CVACCESS]0[EEEEEEEE]1
                AAAAAA AAA1DDD = Output Address
                AAAAAA AAA0000 = Decoder Address
                CVACCESS = DCC Programming CMD
                EEEEEEEE = Checksum

                CVACCESS [1110CCVV]0[VVVVVVVV]0[DDDDDDDD]
                CC = Command
                CC = 01 Verify Byte
                CC = 11 Write Byte
                CC = 10 Bit Manipulation
                VV VVVVVVVV = CV Number
                DDDDDDDD = New Value
                EEEEEEEE = Checksum
            */

      decoderAddress = (((~Msg->Data[1]) & 0b01110000) << 2) + (Msg->Data[0] & 0b00111111);  // Adresse ist in 2 Bytes kodiert, MSB invertiert in Byte1 (Bit4-6) und der Rest in Byte0 (Bit 0-5)
      command = Msg->Data[2] & 0b00001100;                                                   // 2 Bits enthalten den Schreib-/ Verify-Befehl etc.
      CV_address = ((Msg->Data[2] & 0b00000011) * 256) + Msg->Data[3] + 1;                   // Nummer der CV
      CV_value = Msg->Data[4];                                                               // CV-Wert für das Schreiben

      if (!(((decoderAddress == decoderAddress_alt) && (CV_address == CV_address_alt) && (command == command_alt)) && puffern_CV))  // immer ausgeben
      {
        switch (command) {
          case 12:  //B1100 --> 12 ==> Schreiben

            SerialBT.print(F("Acc   "));
            if (decoderAddress < 100) SerialBT.print(" ");
            if (decoderAddress < 10) SerialBT.print(" ");
            SerialBT.print(decoderAddress);

            SerialBT.print("   CV ");
            SerialBT.print(CV_address);
            SerialBT.print(" ");
            if (CV_address < 100) SerialBT.print(" ");
            if (CV_address < 10) SerialBT.print(" ");

            SerialBT.print(F("Schreibe CV ="));
            if (CV_value < 100) SerialBT.print(" ");
            if (CV_value < 10) SerialBT.print(" ");
            SerialBT.print(" ");
            SerialBT.print(CV_value);
            print_spaces(33);
            break;

          case 4:  //B0100 -->  4 ==> Lesen

            SerialBT.print(F("Acc   "));
            if (decoderAddress < 100) SerialBT.print(" ");
            if (decoderAddress < 10) SerialBT.print(" ");
            SerialBT.print(decoderAddress);

            SerialBT.print("   CV ");
            SerialBT.print(CV_address);
            SerialBT.print(" ");
            if (CV_address < 100) SerialBT.print(" ");
            if (CV_address < 10) SerialBT.print(" ");
            SerialBT.print(F("Lese CV "));
            print_spaces(42);
            break;

          case 8:  //B1000 ==> Bit-Gefummel !

            // ... 111K-DBBB EEEE-EEEE --> Zugriffe auf einzelne Bits
            // K = 1 – Bit Schreiben
            // K = 0 – Bit Überprüfen

            if (Msg->Data[pktByteCount - 2] & 0b00010000) {
              SerialBT.print("Schreibe Bit #");
              SerialBT.print(Msg->Data[pktByteCount - 2] & 0b00000111);
              SerialBT.print(" = ");
              SerialBT.print((Msg->Data[pktByteCount - 2] & 0b00001000) >> 3);
              print_spaces(36);
            } else {
              SerialBT.print("Lese    Bit #");
              SerialBT.print(Msg->Data[2] & 0b00000111);
              print_spaces(41);
            }
            break;
        }
        printPacket(Msg);

        decoderAddress_alt = decoderAddress;
        CV_address_alt = CV_address;
        command_alt = command;
      }
      return;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  else if (decoderType == 0)  // --> Lok / Funktionsdekoder
  {
    /*  Aufbau der Fahrbefehle:
            Fahrbefehl, kurze Adressen
                Byte 7 6 5 4 3 2 1 0
                ---------------------------------------------------------------------------
                0    0 A A A A A A A    Adresse 7 Bit
                1    0 1 R S S S S S    R = Fahrtrichtung, Sn = Geschwindigkeit 28 Stufen
                2          XOR          Prüfbyte

                0    0 A A A A A A A    Adresse 7 Bit
                1    0 0 1 1 1 1 1 1    Befehlsbyte 0x3F
                2    R S S S S S S S    R = Fahrtrichtung, Sn = Geschwindigkeit 127 Stufen
                3          XOR          Prüfbyte

                - Fahrbefehl 28 Stufen: unsigned char speed = ((Byte[1] & 0x0F) << 1) + ((Byte[1] & 0x10) >> 4);
                - Fahrbefehl 127 Stufen: unsigned char speed = Byte[2] & 0x7F;

                Fahrbefehl, lange Adressen
                Byte 7 6 5 4 3 2 1 0
                ---------------------------------------------------------------------------
                0    1 1 A A A A A A    Adresse 6 Bit
                1    A A A A A A A A    Adresse 8 Bit
                2    0 1 R S S S S S    R = Fahrtrichtung, Sn = Geschwindigkeit 28 Stufen
                3          XOR          Prüfbyte

                Byte 7 6 5 4 3 2 1 0
                ---------------------------------------------------------------------------
                0    1 1 A A A A A A    Adresse 6 Bit
                1    A A A A A A A A    Adresse 8 Bit
                2    0 0 1 1 1 1 1 1    Befehlsbyte 0x3F
                3    R S S S S S S S    R = Fahrtrichtung, Sn = Geschwindigkeit 127 Stufen
                4          XOR          Prüfbyte

                - Fahrbefehl 28 Stufen: unsigned char speed = ((Byte[2] & 0x0F) << 1) + ((Byte[2] & 0x10) >> 4);
                - Fahrbefehl 127 Stufen: unsigned char speed = Byte[3] & 0x7F;

        */
    if (Anzeige_Loks)
    // zu Testzwecken filtern auf bestimmte Lok-Adresse
    // if (Anzeige_Loks && (decoderAddress == 42 || decoderAddress == 888))
    // if (Anzeige_Loks && (decoderAddress < 25))
    {
      byte instructionType = Befehls_Byte >> 5;

      /*  Bits 7-6-5 sind relevant, ergibt 0 - 7 =  8 unterschiedliche Befehlstypen
                0 = Control ??
                1 = 0011-1111 128 Geschwindigkeitsstufen-Befehl, 0011-1110 Sonderbetriebsarten-Befehl
                2 = 01xx-xxxx Basis Geschwindigkeits- und Richtungsbefehl rückwärts 28 Stufen
                3 = 01xx-xxxx Basis Geschwindigkeits- und Richtungsbefehl vorwärts  28 Stufen
                4 = Lok Funktionen F0, F1 - F4
                5 = Lok-Funktionen F5 - F12
                6 = Lok-Funktionen F13 - F36
                7 = CVs                                 */

      if (puffern_Lok) {
#ifdef debug
        // alle Werte im Puffer ausgeben
        SerialBT.print(F("1. Befehl erkannt, Stand Puffer vor der Verarbeitung:"));

        for (byte j = 0; j < bufferSizeLok; j++) {
          SerialBT.print(j);
          SerialBT.print(" = ");
          SerialBT.print(Lok_received[j].ADR, HEX);
          SerialBT.print(" | ");
          SerialBT.print(Lok_received[j].ORDER, HEX);
          SerialBT.print(" | ");
          SerialBT.println(Lok_received[j].FUNC, HEX);
        }
#endif
        // Schauen im Array, ob die gefundenen Bytes schon einmal ausgegeben wurden
        if (Lok_counter > 0) {
          for (byte j = 0; j < Lok_counter; j++) {
            Paket_bekannt_L = 0;
#ifdef debug
            SerialBT.println("2. Prüfen, ob neu oder bekannt:");
            SerialBT.print(j);
            SerialBT.print(" = ");
            SerialBT.print(Lok_received[j].ADR, HEX);
            SerialBT.print(" | ");
            SerialBT.print(decoderAddress, HEX);
            SerialBT.print(" |    ");
            SerialBT.print(Lok_received[j].ORDER, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Befehl, HEX);
            SerialBT.print(" |    ");
            SerialBT.print(Lok_received[j].FUNC, HEX);
            SerialBT.print(" | ");
            SerialBT.print(Funktion, HEX);
            SerialBT.println(" | ");
#endif
            if (Lok_received[j].ADR == decoderAddress) {
              Paket_bekannt_L++;
            }
            if (Lok_received[j].ORDER == Befehl) {
              Paket_bekannt_L++;
            }
            if (Lok_received[j].FUNC == Funktion) {
              Paket_bekannt_L++;
            }

            if (Paket_bekannt_L == 3) {
#ifdef debug
              SerialBT.print(F("--> Paket bereits bekannt"));
              SerialBT.println(" ");
#endif
              return;  // nix machen, keine Ausgabe im seriellen Monitor: war lediglich eine Wiederholung !
            }
          }
        }
#ifdef debug
        SerialBT.println();
        SerialBT.println(F("Neuer Befehl erkannt !"));
#endif
        if (Lok_counter > 0) {
          // Pufferzeilen mit der gleichen Adresse und dem gleichen Befehl rauslöschen
          for (byte j = 0; j < Lok_counter; j++) {
            if (Befehl != 0b00111101)  // bei allen Fahrzeugbefehlen ohne Analoggruppe
            {
              if (Lok_received[j].ADR == decoderAddress && Lok_received[j].ORDER == Befehl) {
                for (byte k = j; k < min(Lok_counter, bufferSizeLok - 1); k++) {
                  Lok_received[k].ADR = Lok_received[k + 1].ADR;
                  Lok_received[k].ORDER = Lok_received[k + 1].ORDER;
                  Lok_received[k].FUNC = Lok_received[k + 1].FUNC;
                }

                Lok_counter--;  // ein Eintrag wurde gelöscht

#ifdef debug
                SerialBT.println();
                SerialBT.print(F("3. 1 Pufferzeile gelöscht: "));
                SerialBT.println(decoderAddress, HEX);
#endif
                break;
              }
            } else  // bei einem Analoggruppenbefehl nur dann alte Einträge löschen, wenn alle 3 Felder gleich sind
            {
              // alle 3 Werte prüfen wg. der Sonderlocke von ESU "Analoggruppe"

              if (Lok_received[j].ADR == decoderAddress && Lok_received[j].ORDER == Befehl && Lok_received[j].FUNC == Funktion) {
                for (byte k = j; k < min(Lok_counter, bufferSizeLok - 1); k++) {
                  Lok_received[k].ADR = Lok_received[k + 1].ADR;
                  Lok_received[k].ORDER = Lok_received[k + 1].ORDER;
                  Lok_received[k].FUNC = Lok_received[k + 1].FUNC;
                }

                Lok_counter--;  // ein Eintrag wurde gelöscht

#ifdef debug
                SerialBT.println();
                SerialBT.print(F("3. 1 Pufferzeile gelöscht: "));
                SerialBT.println(decoderAddress, HEX);
#endif
                break;
              }
            }
          }
        }

        // alle Werte im Puffer ausgeben
#ifdef debug
        SerialBT.println();
        SerialBT.println(F("4. Stand Puffer nach Löschen:"));

        for (byte j = 0; j < bufferSizeLok; j++) {
          SerialBT.print(j);
          SerialBT.print(" = ");
          SerialBT.print(Lok_received[j].ADR, HEX);
          SerialBT.print(" | ");
          SerialBT.print(Lok_received[j].ORDER, HEX);
          SerialBT.print(" | ");
          SerialBT.println(Lok_received[j].FUNC, HEX);
        }
#endif

        // eine Zeile im Acc_received mit neuem Paket befüllen
        if (Lok_counter < bufferSizeLok) {
          Lok_received[Lok_counter].ADR = decoderAddress;
          Lok_received[Lok_counter].ORDER = Befehl;
          Lok_received[Lok_counter].FUNC = Funktion;
#ifdef debug
          SerialBT.println();
          SerialBT.println(F("5. Fülle Puffer:"));
          SerialBT.print(Lok_counter);
          SerialBT.print(" = ");
          SerialBT.print(Lok_received[Lok_counter].ADR, HEX);
          SerialBT.print(" | ");
          SerialBT.print(Lok_received[Lok_counter].ORDER, HEX);
          SerialBT.print(" | ");
          SerialBT.print(Lok_received[Lok_counter].FUNC, HEX);
          SerialBT.println(F(" --> Neuer Eintrag"));
#endif
          Lok_counter++;
        } else {
          // ersten ältesten Wert löschen und alle anderen Wertepaare nach links rücken
          for (byte j = 0; j < bufferSizeLok - 1; j++) {
            Lok_received[j].ADR = Lok_received[j + 1].ADR;
            Lok_received[j].ORDER = Lok_received[j + 1].ORDER;
            Lok_received[j].FUNC = Lok_received[j + 1].FUNC;
          }
          // den letzten Eintrag nun mit dem neuen Paket befüllen

          Lok_received[bufferSizeLok - 1].ADR = decoderAddress;
          Lok_received[bufferSizeLok - 1].ORDER = Befehl;
          Lok_received[bufferSizeLok - 1].FUNC = Funktion;

          Lok_counter = bufferSizeLok;
        }
        // alle Werte im Puffer ausgeben
#ifdef debug
        SerialBT.println();
        SerialBT.println(F("6. Stand Puffer nach der Verarbeitung:"));

        for (byte j = 0; j < bufferSizeLok; j++) {
          SerialBT.print(j);
          SerialBT.print(" = ");
          SerialBT.print(Lok_received[j].ADR, HEX);
          SerialBT.print(" | ");
          SerialBT.print(Lok_received[j].ORDER, HEX);
          SerialBT.print(" | ");
          SerialBT.println(Lok_received[j].FUNC, HEX);
        }
#endif
      }

      //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
      SerialBT.print("Lok ");
      if (decoderAddress < 10) SerialBT.print("    ");
      else if (decoderAddress < 100) SerialBT.print("   ");
      else if (decoderAddress < 1000) SerialBT.print("  ");
      else if (decoderAddress < 10000) SerialBT.print(" ");
      SerialBT.print(decoderAddress);
      SerialBT.print("   ");
      Lokname(decoderAddress);
      SerialBT.print(" ");
      //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
      switch (instructionType) {
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 0:
          // 000x-xxxx
          SerialBT.print(" Control ");
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 1:
          // Advanced Operations
          // 001x-xxxx

          if (Befehls_Byte == 0b00111111)  //128 speed steps
          {
            //0011-1111 128 Geschwindigkeitsstufen-Befehl

            // Richtung auswerten Bit 7 "R" im vorletzten Byte
            if (bitRead(Msg->Data[pktByteCount - 2], 7)) SerialBT.print("  -->> ");
            else SerialBT.print("  <<-- ");

            // Geschwindigkeit Bit 6 - 0 vom vorletzten Byte
            byte speed = Msg->Data[pktByteCount - 2] & 0b01111111;
            if (speed == 0) SerialBT.print(" Stopp  ");       // wenn = 0, dann Stoppbefehl
            else if (speed == 1) SerialBT.print(" Nothalt");  // wenn = 1, dann Notbremse
            else {
              SerialBT.print(speed - 1);  // 1 abziehen; Stufen gehen von 2 - 127 = 126 Stufen
              if (speed - 1 < 10) SerialBT.print("       ");
              else if (speed - 1 < 100) SerialBT.print("      ");
              else SerialBT.print("     ");
            }
            print_spaces(25);
          }
          //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
          else if (Befehls_Byte == 0b00111110)  //Speed Restriction
          {
            //0011-1110 Sonderbetriebsarten-Befehl
            if (bitRead(Msg->Data[pktByteCount - 2], 7)) SerialBT.print(" On ");
            else SerialBT.print(" Off ");
            SerialBT.print(Msg->Data[pktByteCount - 1]) & 0b01111111;
            print_spaces(22);
          }

          // NEU 12.12.2020
          else if (Befehls_Byte == 0b00111101) {
            /*
                             Analogfunktionsgruppe

                             Dieser Befehl ist drei Byte lang und hat das Format: 0011-1101 SSSS-SSSS DDDD-DDDD
                             Er ist bestimmt, um bis zu 256 Analogkanäle zu steuern. Dabei legt SSSS-SSSS im zweiten Befehlsbyte den Analogfunktionsausgang (Steuerkanal) (0-255) und das dritte
                             Befehlsbyte DDDD-DDDD die Analogfunktionsdaten Daten (0-255) fest.

                             Verwendung der Analogausgänge:
                             SSSS-SSSS = 0000-0001 - Lautstärkesteuerung
                             SSSS-SSSS = 0001-0000 bis 0001-1111 - Positionssteuerung

                             Alle oben nicht definierten Werte von SSSS-SSSS zwischen 0000-0000 und 0111-1111 einschließlich sind reserviert.
                             Die Werte von 1000-0000 bis 1111-1111 können beliebig verwendet werden. Diese Funktion darf nicht zur Geschwindigkeitssteuerung eines Fahrzeugdecoders verwendet werden.
                        */
            SerialBT.print("  Analogfkt.gruppe Steuerkanal ");
            if (Msg->Data[2] < 100) SerialBT.print(" ");
            if (Msg->Data[2] < 10) SerialBT.print(" ");
            SerialBT.print(Msg->Data[2]);
            SerialBT.print(" = ");
            if (Msg->Data[3] < 100) SerialBT.print(" ");
            if (Msg->Data[3] < 10) SerialBT.print(" ");
            SerialBT.print(Msg->Data[3]);
          }
          // NEU 12.12.2020
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 2:
          // Reverse speed step 28 Stufen
          // 01xx-xxxx Basis Geschwindigkeits- und Richtungsbefehl
          //                    speed = ((Befehls_Byte & 0b00001111) << 1) - 3 + bitRead(Befehls_Byte, 4);
          speed = ((Befehls_Byte & 0b00001111) << 1) + ((Befehls_Byte & 0x10) >> 4) - 3;
          if (speed == 253 || speed == 254) SerialBT.print("  Stopp  ");
          else if (speed == 255 || speed == 0) SerialBT.print("  Nothalt");
          else {
            SerialBT.print("  <<-- ");
            SerialBT.print(speed);
            if (speed < 10) SerialBT.print(" ");
          }
          print_spaces(31);
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 3:  // Forward speed step 28 Stufen
          //                   speed = ((Befehls_Byte & 0b00001111) << 1) - 3 + bitRead(Befehls_Byte, 4);
          speed = ((Befehls_Byte & 0b00001111) << 1) + ((Befehls_Byte & 0x10) >> 4) - 3;
          if (speed == 253 || speed == 254) SerialBT.print("  Stopp  ");
          else if (speed == 255 || speed == 0) SerialBT.print("  Nothalt");
          else {
            SerialBT.print("  -->> ");
            SerialBT.print(speed);
            if (speed < 10) SerialBT.print(" ");
          }
          print_spaces(31);
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 4:  // Loc Function L-4-3-2-1
          // 100x-xxxx Funktionssteuerung F0-F4, die F-Nummern sind allerdings seltsam verteilt ;-)

          if (Funktion & 16) SerialBT.print("   F0");
          else SerialBT.print("   f0");

          for (byte k = 0; k < 4; k++) {
            if (bitRead(Funktion, k)) SerialBT.print("   F");
            else SerialBT.print("   f");
            SerialBT.print(k + 1);
          }
          print_spaces(15);
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 5:  // Loc Function 8-7-6-5
          // 1011-xxxx Funktionssteuerung F5-F8
          if (bitRead(Befehls_Byte, 4)) {
            for (byte k = 0; k < 4; k++) {
              if (bitRead(Funktion, k)) SerialBT.print("   F");
              else SerialBT.print("   f");
              SerialBT.print(k + 5);
            }
            print_spaces(20);
          }
          //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
          else  // Loc Function 12-11-10-9
          {
            //1010-xxxx Funktionssteuerung F9-F12

            for (byte k = 0; k < 4; k++) {
              if (k == 0) SerialBT.print(" ");
              if (bitRead(Funktion, k)) SerialBT.print("  F");
              else SerialBT.print("  f");
              SerialBT.print(k + 9);
            }
            print_spaces(20);
          }
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 6:  // 110x-xxxx Eigenschaften-Erweiterungs-Befehle

          switch (Befehls_Byte & 0b00011111) {
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
            case 0:  // Binary State Control Instruction long form
              // 1100-0000 Binärzustandssteuerungsbefehl lange Form
              SerialBT.print(" BinStateLong ");
              SerialBT.print(256 * Msg->Data[pktByteCount - 1] + (Msg->Data[pktByteCount - 2] & 0b01111111));
              if bitRead (Msg->Data[pktByteCount - 2], 7) SerialBT.print(" On ");
              else SerialBT.print(" Off ");
              break;
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
            case 0b00011101:  // Binary State Control
              // 1101-1101 Binärzustandssteuerungsbefehl kurze Form
              SerialBT.print(" BinStateShort ");
              SerialBT.print(Msg->Data[pktByteCount - 1] & 0b01111111);
              if bitRead (Msg->Data[pktByteCount - 1], 7) SerialBT.print(" On ");
              else SerialBT.print(" Off ");
              break;
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
            case 0b00011110:  // F13-F20 Function Control
              // 1101-1110 Funktionssteuerung F13-F20

              for (byte k = 0; k < 8; k++) {
                if (bitRead(Funktion, k)) SerialBT.print("  F");
                else SerialBT.print("  f");
                SerialBT.print(k + 13);
              }
              break;
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
            case 0b00011111:  // F21-F28 Function Control
              //1101-1111 Funktionssteuerung F21-F28

              for (byte k = 0; k < 8; k++) {
                if (bitRead(Funktion, k)) SerialBT.print("  F");
                else SerialBT.print("  f");
                SerialBT.print(k + 21);
              }
              break;
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
            case 0b00011000:  // F29-F36 Function Control
              //1101-1000 Funktionssteuerung F29 - F36
              for (byte k = 0; k < 8; k++) {
                if (bitRead(Funktion, k)) SerialBT.print("  F");
                else SerialBT.print("  f");
                SerialBT.print(k + 29);
              }
              break;
          }
          break;
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 7:
          SerialBT.print("  CV ");
          if (Befehls_Byte & 0b00010000)  // CV Short Form
          {
            byte cvType = Befehls_Byte & 0b00001111;
            switch (cvType) {
              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00000010:
                // KKKK = 0010 – Beschleunigungswert in einer Mehrfachtraktion (CV #23)
                SerialBT.print("23 ");
                SerialBT.print(Msg->Data[pktByteCount - 1]);
                break;
              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00000011:
                // KKKK = 0011 – Verzögerungswert in einer Mehrfachtraktion (CV #24)
                SerialBT.print("24 ");
                SerialBT.print(Msg->Data[pktByteCount - 1]);
                break;
              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00001001:
                /*
                                    D.2 "Service Mode Decoder Lock Instruction"
                                        Dieser Befehl wird in der Technical Note TN-1-05 der NMRA näher beschrieben.
                                        Die "Service Mode Decoder Lock Instruction" (SMDLI) verhindert die Programmierung einiger Decoder, während andere auf dem
                                        gleichen Gleis programmiert werden können. Es werden dazu Befehle für den Programmiermodus auf dem normalen Fahrgleis verwendet,
                                        womit man nicht den Vorteil der strombegrenzten Testumgebung hat und Decoder, die diesen Befehl nicht unterstützen, ungewollt umprogrammiert werden können.

                                    Das Befehlsformat lautet: {20 Synchronbits} 0 0000-0000 0 1111-1001 0 0AAA-AAAA 0 PPPP-PPPP 1

                                    Dabei steht das AAA-AAAA für die kurze Adresse des Decoders, der weiterhin Befehle des Programmiermodus ausführen wird.
                                    Ein Decoder, der diesen Befehl unterstützt, vergleicht seine Adresse in CV #1 gegen die Adresse in dem Befehl.
                                    Stimmen die Adressen nicht überein, geht er in einen gesperrten Zustand über, in dem er auch nach Aus- und wieder Einschalten der
                                    Spannung verbleibt. In diesem Zustand ignoriert er alle Befehle des Programmiermodus.
                                */
                SerialBT.print(F("Decoder Lock "));
                SerialBT.print(Msg->Data[pktByteCount - 1]);
                break;
            }
          }
          //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
          else  // CV Long Form für Betriebsmodus  POM !
          {
            CV_address = ((Msg->Data[pktByteCount - 4] & 0b00000011) * 256) + Msg->Data[pktByteCount - 3] + 1;
            SerialBT.print(CV_address);

            if (CV_address < 1000) SerialBT.print(" ");
            if (CV_address < 100) SerialBT.print(" ");
            if (CV_address < 10) SerialBT.print(" ");

            switch (Befehls_Byte & 0b00001100) {
              /*  Die für den Befehlstyp (xxxx-KKxx) im ersten Befehlsbyte festgelegten Werte sind:
                                  KK = 00 – reserviert
                                  KK = 01 – Byte Überprüfen
                                  KK = 11 – Byte Schreiben
                                  KK = 10 – Bit Manipulation */

              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00000100:  // Lesen Byte

                SerialBT.print(F("Lese CV"));
                print_spaces(24);
                break;

              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00001100:  // Schreiben Byte

                SerialBT.print(F("Schreibe CV ="));
                if (Msg->Data[pktByteCount - 2] < 100) SerialBT.print(" ");
                if (Msg->Data[pktByteCount - 2] < 10) SerialBT.print(" ");
                SerialBT.print(" ");
                SerialBT.print(Msg->Data[pktByteCount - 2]);
                print_spaces(14);
                break;

              //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
              case 0b00001000:  //B1000 ==> Bit-Gefummel !

                // ... 111K-DBBB EEEE-EEEE --> Zugriffe auf einzelne Bits
                // K = 1 – Bit Schreiben
                // K = 0 – Bit Überprüfen

                if (Msg->Data[pktByteCount - 2] & 0b00010000) {
                  SerialBT.print("Schreibe Bit #");
                  SerialBT.print(Msg->Data[pktByteCount - 2] & 0b00000111);
                  SerialBT.print(" = ");
                  SerialBT.print((Msg->Data[pktByteCount - 2] & 0b00001000) >> 3);
                  print_spaces(20);
                } else {
                  SerialBT.print("Lese    Bit #");
                  SerialBT.print(Msg->Data[2] & 0b00000111);
                  print_spaces(25);
                }
                break;
            }
          }
          break;
      }
      printPacket(Msg);
    }
  }
}

//-------------------------------------------------------------------------------------------------------
void notifyCVChange(unsigned int CvAddr, byte Value)
//-------------------------------------------------------------------------------------------------------
// Wird aufgerufen, nachdem ein CV-Wert verändert wurde
{
  SerialBT.print("notifyCVChange: ");
  SerialBT.print("CV");
  SerialBT.print(CvAddr);
  SerialBT.print(" = ");
  SerialBT.println(Value);
}

//-------------------------------------------------------------------------------------------------------
void Byte_to_Bits(byte b)
//-------------------------------------------------------------------------------------------------------
// Gibt einen Byte-Wert binär mit führenden Nullen und "-" nach der 4. Stelle aus
// z.B. 12 --> 0001 1000
{
  for (int i = 7; i >= 0; i--) {
    SerialBT.print(bitRead(b, i));
    if (i == 4) SerialBT.print("-");
  }
}

//-------------------------------------------------------------------------------------------------------
void print_spaces(byte b)
//-------------------------------------------------------------------------------------------------------
// Gibt b mal ein Leerzeichen aus zur Formatierung
{
  for (byte i = 0; i < b; i++) {
    SerialBT.print(" ");
  }
}


//-------------------------------------------------------------------------------------------------------
void Lokname(unsigned int DCC_Adresse)
//-------------------------------------------------------------------------------------------------------
// Ausgabe der Loknamen zu den gefundenen Adressen
{
  switch (DCC_Adresse) {
    case 8: SerialBT.print(F("Class 08 Hornby ")); break;
    case 10: SerialBT.print(F("Köf Arnold      ")); break;
    case 20: SerialBT.print(F("V20 Beckmann    ")); break;
    case 23: SerialBT.print(F("V23 Piko        ")); break;
    case 36: SerialBT.print(F("V36 Tillig      ")); break;
    case 55: SerialBT.print(F("BR55 Piko       ")); break;
    case 56: SerialBT.print(F("BR56 BTTB       ")); break;
    case 60: SerialBT.print(F("V60 JaTT        ")); break;
    case 75: SerialBT.print(F("BR75 Beckmann   ")); break;
    case 80: SerialBT.print(F("BR80 Roco       ")); break;
    case 86: SerialBT.print(F("BR86 Tillig     ")); break;
    case 89: SerialBT.print(F("BR89 JaTT       ")); break;
    case 91: SerialBT.print(F("BR91 JaTT       ")); break;
    case 92: SerialBT.print(F("BR92 BTTB       ")); break;
    case 94: SerialBT.print(F("BR94 Beckmann   ")); break;
    case 98: SerialBT.print(F("BR98 Jenker     ")); break;
    case 110: SerialBT.print(F("BR110 Tillig    ")); break;
    case 118: SerialBT.print(F("BR118 Tillig    ")); break;
    case 133: SerialBT.print(F("VT133 Peresvet  ")); break;
    case 135: SerialBT.print(F("VT135 KRES      ")); break;
    case 172: SerialBT.print(F("LVT 2 BTTB      ")); break;
    case 260: SerialBT.print(F("BR260 Kühn      ")); break;
    case 312: SerialBT.print(F("BR312 Piko      ")); break;
    case 1206: SerialBT.print(F("MAK G1206 Piko  ")); break;
    case 10000: SerialBT.print(F("ECoS Dummybefehl")); break;
    default: SerialBT.print(F("???             "));  // 16 Leerzeichen, falls Lokname nicht gefunden !
  }
}

//-------------------------------------------------------------------------------------------------------
void print_Zahl_rechts_ln(unsigned long zahl)
//-------------------------------------------------------------------------------------------------------
// Gibt eine ganze Zahl rechtsbündig aus
{
  SerialBT.print(" ");
  if (zahl < 10000000) SerialBT.print(" ");
  if (zahl < 1000000) SerialBT.print(" ");
  if (zahl < 100000) SerialBT.print(" ");
  if (zahl < 10000) SerialBT.print(" ");
  if (zahl < 1000) SerialBT.print(" ");
  if (zahl < 100) SerialBT.print(" ");
  if (zahl < 10) SerialBT.print(" ");
  SerialBT.println(zahl);
}


//-------------------------------------------------------------------------------------------------------
void printPacket(DCC_MSG* Msg)
//-------------------------------------------------------------------------------------------------------
// Gibt alle gefundenen Bytes eines DCC-Befehls aus, ohne Präambel, ohne Prüfbyte
{
  SerialBT.print(" ");
  for (byte n = 0; n < pktByteCount - 1; n++)  // wenn man das "- 1" löscht, wird auch das Prüfbyte angezeigt!
  {
    SerialBT.print("  ");
    Byte_to_Bits(Msg->Data[n]);
  }
  SerialBT.println(" ");
}
