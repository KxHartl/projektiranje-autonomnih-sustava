ros2_control Mock Sustav za Fanuc M10iA Robot
Opis projekta
Ovaj projekt implementira ros2_control mock sustav za 6-stupanjski Fanuc M10iA industrijski robot. Mock sustav omogućuje simulaciju i testiranje upravljanja robotom bez potrebe za fizičkim hardverom.

Projekt uključuje:

Prošireni URDF/xacro opis robota s ros2_control oznakama

Konfiguraciju controllera (forward_position_controller i joint_trajectory_controller)

Četiri launch datoteke za različite scenarije testiranja

Vizualizaciju u RViz-u s GUI kontrolom

Preduvjeti
Sustav
Ubuntu 22.04

ROS 2 Humble

Potrebni ROS 2 paketi
Instalirajte sljedeće pakete:

bash
sudo apt update
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-xacro
sudo apt install ros-humble-rviz2
Instalacija
1. Kreiranje radnog prostora
bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
2. Preuzimanje projekta
bash
git clone https://github.com/KxHartl/projektiranje-autonomnih-sustava.git
cd projektiranje-autonomnih-sustava/student-assignment-01
3. Instalacija zavisnosti
bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
4. Buildanje paketa
bash
cd ~/ros2_ws
colcon build --symlink-install
5. Sourcing okruženja
bash
source ~/ros2_ws/install/setup.bash
Napomena: Dodajte ovu naredbu u ~/.bashrc kako biste automatski sourcali okruženje pri svakom otvaranju novog terminala:

bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
Struktura projekta
text
student-assignment-01/
├── config/
│   └── fanuc_controllers.yaml       # Konfiguracija controllera
├── launch/
│   ├── view_robot.launch.py         # Launch 1: Vizualizacija s GUI kontrolom
│   ├── robot_controller.launch.py   # Launch 2: Robot s controllerima
│   ├── forward_position.launch.py   # Launch 3: Slanje pozicija
│   └── trajectory_control.launch.py # Launch 4: Slanje trajektorija
├── urdf/
│   ├── fanuc_m10ia.urdf.xacro      # Glavni URDF opis robota
│   ├── fanuc_m10ia_macro.xacro     # Makro definicije
│   └── fanuc.ros2_control.xacro    # ros2_control konfiguracija
└── meshes/
    └── visual/                      # Mesh datoteke za vizualizaciju
Launch datoteke
Projekt sadrži četiri launch datoteke koje omogućuju različite načine testiranja i upravljanja robotom.

Launch 1: Vizualizacija robota s GUI kontrolom
Ova launch datoteka pokreće samo vizualizaciju robota u RViz-u s grafičkim sučeljem za ručno upravljanje zglobovima.

Pokretanje:

bash
ros2 launch student-assignment-01 view_robot.launch.py
Što se pokreće:

robot_state_publisher - objavljuje transformacije robota

joint_state_publisher_gui - GUI za ručno postavljanje pozicija zglobova

rviz2 - vizualizacija robota

Korištenje:

U GUI prozoru pomičite klizače za svaki zglob

Robot će se vizualno ažurirati u RViz-u

Ova launch datoteka ne koristi ros2_control sustav

Launch 2: Robot s controllerima
Ova launch datoteka pokreće ros2_control mock sustav s oba controllera (forward_position_controller i joint_trajectory_controller).

Pokretanje:

bash
ros2 launch student-assignment-01 robot_controller.launch.py
Što se pokreće:

ros2_control_node - glavni čvor za upravljanje

joint_state_broadcaster - objavljuje stanja zglobova

forward_position_controller - aktiviran (za direktno upravljanje pozicijama)

joint_trajectory_controller - učitan ali neaktivan (za upravljanje trajektorijama)

robot_state_publisher - objavljuje transformacije

rviz2 - vizualizacija

Provjera aktivnih controllera:

bash
ros2 control list_controllers
Prebacivanje između controllera:

bash
# Deaktivirati forward_position_controller i aktivirati joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Vratiti na forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
Launch 3: Slanje pozicija forward_position_controlleru
Ova launch datoteka objavljuje pozicije za sve zglobove koristeći forward_position_controller.

Prije pokretanja:
Prvo pokrenite Launch 2 i osigurajte da je forward_position_controller aktivan.

Pokretanje (u novom terminalu):

bash
ros2 launch student-assignment-01 forward_position.launch.py
Što radi:

Objavljuje niz pozicija za svih 6 zglobova

Robot se kreće prema zadanim pozicijama

Pozicije se šalju na topic /forward_position_controller/commands

Ručno slanje pozicija:

bash
# Pomicanje svih zglobova u neutralnu poziciju
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Pomicanje u testnu poziciju
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.8, -0.2, 0.4, -0.1]"

# Povratak u home poziciju
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
Napomena: Sve vrijednosti su u radijanima i moraju biti unutar granica zglobova definirane u URDF datoteci.

Launch 4: Slanje trajektorija joint_trajectory_controlleru
Ova launch datoteka objavljuje trajektorije koristeći joint_trajectory_controller.

Prije pokretanja:

Pokrenite Launch 2

Prebacite na joint_trajectory_controller:

bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
Pokretanje (u novom terminalu):

bash
ros2 launch student-assignment-01 trajectory_control.launch.py
Što radi:

Objavljuje kompletnu trajektoriju s više točaka

Svaka točka ima definirane pozicije, brzine i vremena

Kontroler interpolira između točaka za glatko kretanje

Ručno slanje trajektorije:

bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 0, nanosec: 0}
    },
    {
      positions: [0.5, -0.5, 0.5, -0.5, 0.5, -0.5],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    },
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
Testiranje
Osnovni test workflow
Test 1: Vizualizacija s GUI-jem
bash
# Terminal 1: Pokreni vizualizaciju
ros2 launch student-assignment-01 view_robot.launch.py
Očekivani rezultat:

Otvara se RViz s modelom robota

Otvara se GUI prozor s klizačima za sve zglobove

Pomicanjem klizača robot se vizualno kreće

Test 2: Forward Position Controller
bash
# Terminal 1: Pokreni controllere
ros2 launch student-assignment-01 robot_controller.launch.py

# Terminal 2: Provjeri status controllera
ros2 control list_controllers

# Terminal 3: Pošalji pozicije
ros2 launch student-assignment-01 forward_position.launch.py
Očekivani rezultat:

Robot se kreće prema zadanim pozicijama

Kretanje je trenutačno (bez interpolacije)

U RViz-u se vidi pomicanje robota

Test 3: Joint Trajectory Controller
bash
# Terminal 1: Pokreni controllere (ako već nije)
ros2 launch student-assignment-01 robot_controller.launch.py

# Terminal 2: Prebaci na trajectory controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

# Terminal 3: Pošalji trajektoriju
ros2 launch student-assignment-01 trajectory_control.launch.py
Očekivani rezultat:

Robot se glatko kreće kroz definiranu trajektoriju

Interpolacija između točaka je vidljiva

Trajektorija se izvršava u zadanom vremenu

Napredni testovi
Provjera topica
bash
# Prikaži sve aktivne topice
ros2 topic list

# Prikaži podatke sa joint_states topica
ros2 topic echo /joint_states

# Prikaži podatke sa dynamic_joint_states topica
ros2 topic echo /dynamic_joint_states
Provjera controllera
bash
# Prikaži sve controllere
ros2 control list_controllers

# Prikaži detalje o specific controlleru
ros2 control list_hardware_interfaces

# Učitaj controller
ros2 control load_controller <controller_name>

# Aktiviraj controller
ros2 control set_controller_state <controller_name> active

# Deaktiviraj controller
ros2 control set_controller_state <controller_name> inactive
Monitoring performansi
bash
# Provjeri frekvenciju objava joint_states
ros2 topic hz /joint_states

# Provjeri frekvenciju command topica
ros2 topic hz /forward_position_controller/commands
ros2 topic hz /joint_trajectory_controller/joint_trajectory
Otklanjanje grešaka
Problem 1: Launch datoteka ne može pronaći paket
Greška:

text
Package 'student-assignment-01' not found
Rješenje:

bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
Problem 2: Controller ne može se učitati
Greška:

text
Failed to load controller
Provjere:

Provjerite je li YAML konfiguracijska datoteka ispravna

Provjerite odgovara li ime controllera u YAML i launch datoteci

Provjerite jesu li imena zglobova u YAML datoteci identična onima u URDF-u

bash
# Provjera konfiguracijske datoteke
cat ~/ros2_ws/src/student-assignment-01/config/fanuc_controllers.yaml

# Provjera dostupnih controllera
ros2 control list_controller_types
Problem 3: Robot se ne kreće
Mogući uzroci:

Controller nije aktivan

Pogrešan format podataka

Vrijednosti izvan granica zglobova

Rješenje:

bash
# Provjeri status controllera
ros2 control list_controllers

# Provjeri dostupna command sučelja
ros2 control list_hardware_interfaces

# Provjeri ograničenja zglobova u URDF datoteci
cat ~/ros2_ws/src/student-assignment-01/urdf/fanuc_m10ia.urdf.xacro | grep limit
Problem 4: RViz ne prikazuje robota
Rješenje:

Dodajte RobotModel prikaz u RViz-u

Provjerite je li Fixed Frame postavljen na "world" ili "base_link"

Provjerite objavljuje li se robot_description topic:

bash
ros2 topic echo /robot_description --once
Problem 5: Mock hardware se ne pokreće
Greška:

text
Failed to initialize hardware
Rješenje:

Provjerite ros2_control xacro datoteku

Osigurajte da je plugin postavljen na mock_components/GenericSystem

Provjerite sintaksu URDF/xacro datoteka:

bash
xacro ~/ros2_ws/src/student-assignment-01/urdf/fanuc_m10ia.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
Važne napomene
ros2_control Mock Sustav
Mock sustav simulira hardversko sučelje bez potrebe za stvarnim robotom. Omogućuje:

Testiranje algoritama upravljanja

Razvoj i debug bez hardvera

Brzo prototipiranje

Sigurno testiranje graničnih slučajeva

Mock sustav čita state_interfaces i command_interfaces definirane u ros2_control xacro datoteci i simulira njihovo ponašanje.

Razlike između Controllera
forward_position_controller:

Direktno prosljeđuje pozicije hardveru

Nema interpolacije između pozicija

Brže izvršavanje

Koristi se za jednostavne pokrete

joint_trajectory_controller:

Interpolira između točaka trajektorije

Glatko kretanje

Podržava pozicije, brzine i akceleracije

Koristi PID kontroler za praćenje trajektorije

Preporučuje se za složenije pokrete

Najbolje prakse
Uvijek sourcajte okruženje prije pokretanja ROS 2 naredbi

Provjerite status controllera prije slanja naredbi

Koristite razumne vrijednosti unutar granica zglobova

Testirajte u malom opsegu prije velikih pokreta

Koristite trajectory controller za proizvodne aplikacije

Spremajte RViz konfiguracije za buduće sesije

Dodatni resursi
Službena dokumentacija
ros2_control dokumentacija

ros2_controllers dokumentacija

Example 7: 6DOF robot tutorial

Korisni linkovi
ROS 2 Humble dokumentacija

URDF/xacro tutorial

Joint Trajectory Controller

Forward Command Controller

GitHub repozitoriji
ros2_control

ros2_control_demos

ros2_controllers

Autor i kontakt
Projekt: ros2_control Mock Sustav za Fanuc M10iA Robot
Kolegij: Projektiranje autonomnih sustava
GitHub: https://github.com/KxHartl/projektiranje-autonomnih-sustava

Za pitanja i prijedloge otvorite Issue na GitHub repozitoriju.

Licenca
Ovaj projekt je namijenjen isključivo za obrazovne svrhe u sklopu kolegija "Projektiranje autonomnih sustava".