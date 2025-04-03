import rosbag
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2


def plot_points_from_bag(bag_file, topic_name, output_file="../bagfiles/map.png"):
    """
    Plota pontos (x, y) de um tópico em um arquivo .bag usando matplotlib e salva o plot.

    Args:
        bag_file (str): Caminho para o arquivo .bag.
        topic_name (str): Nome do tópico que contém os pontos.
        output_file (str, opcional): Caminho para salvar o plot. Padrão é "points_plot.png".
    """

    try:
        bag = rosbag.Bag(bag_file)
    except rosbag.ROSBagException as e:
        print(f"Erro ao abrir o arquivo .bag: {e}")
        return

    x_points = []
    y_points = []

    try:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            points_list = pc2.read_points_list(msg)
            if points_list:
                for point in points_list:
                    x_points.append(point[0])
                    y_points.append(point[1])
    except AttributeError:
        print(f"Erro: A mensagem no tópico '{topic_name}' não possui os campos 'x' e 'y'.")
        bag.close()
        return
    except Exception as e:
        print(f"Erro ao ler mensagens do tópico '{topic_name}': {e}")
        bag.close()
        return

    bag.close()

    if not x_points or not y_points:
        print(f"Nenhum ponto encontrado no tópico '{topic_name}'.")
        return

    plt.figure()
    plt.plot(x_points, y_points, 'b.')  # Plota os pontos como pontos azuis
    plt.scatter(x_points, y_points)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'Pontos do tópico {topic_name}')
    plt.grid(True)

    # # Define os limites do gráfico
    # plt.xlim(-10, 10)
    # plt.ylim(-10, 10)

    plt.scatter(0, 0, color='red', marker='o', label=f'Referencial: ({0}, {0})')
    plt.legend()

    plt.savefig(output_file)
    plt.show()


if __name__ == "__main__":
    bag_file = "../bagfiles/subset.bag"  
    topic_name = "/lidar_points"  

    plot_points_from_bag(bag_file, topic_name)