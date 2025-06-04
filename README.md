# slow - SLAM Simples com ROS e Sensor IR Rotativo

Este pacote ROS implementa uma solução de mapeamento e navegação para robôs de baixo custo utilizando:

- Sensor infravermelho em servo motor (simulando um LIDAR)
- Estimativa de posição via IMU
- Publicação de mapa 2D com `OccupancyGrid`
- Visualização em RViz

## 📦 Estrutura do Pacote

### Nós ROS incluídos:

| Nó | Descrição |
|----|-----------|
| `imu_odom.py` | Calcula odometria com base em dados de heading da IMU. Publica em `/lena/odom`. |
| `servo_pub.py` | Controla o servo motor, coleta distâncias IR e publica `LaserScan` em `/scan`. |
| `simple_mapper.py` | Gera um mapa de ocupação simples baseado em dados do `LaserScan` e da odometria. Publica em `/simple_map`. |
| `map_builder.py` | Cria uma trilha (trajetória) com base na odometria. Publica `Marker` em `/path_marker`. |

## 🚀 Como Executar

### Pré-requisitos

- ROS Noetic
- Python 3
- Dependências do ROS:
