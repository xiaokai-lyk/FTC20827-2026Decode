
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

#中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# FTC自动任务：Agent结构 vs 传统线性流程

def plot_combined():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    fig.subplots_adjust(hspace=0.4)

    # --- Agent结构流程图（横向步骤+可编辑内容） ---
    ax = ax1
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 12)
    ax.axis('off')

    # 每个步骤的内容列表（可编辑填充）
    perception_list = ["超时检测", "位置传感", "初始化配置检测"]
    decision_list = ["目标选择", "路径规划", "优先级调整"]
    action_list = ["驱动控制", "射击执行", "取球动作"]
    feedback_list = ["位置误差", "时间误差"]

    # 步骤名和横坐标
    steps = [
        ("感知", 2, perception_list),
        ("决策", 6, decision_list),
        ("执行", 10, action_list),
        ("反馈", 14, feedback_list),
    ]
    y_center = 8
    box_w, box_h = 2.2, 1.0
    step_box_h = 1.2

    # 画每个步骤的主框和标题（标题居中，避免出框）
    for step, x, content_list in steps:
        # 步骤大框
        ax.add_patch(Rectangle((x-1.1, y_center+1.5), box_w, step_box_h, edgecolor='navy', facecolor='lightskyblue', lw=2, alpha=0.7))
        y_header = y_center + 1.5 + step_box_h/2
        ax.text(x, y_header, step, ha='center', va='center', fontsize=14, weight='bold')
        # 内容框（垂直排列）
        for i, item in enumerate(content_list):
            y_item = y_center - i*1.2
            ax.add_patch(Rectangle((x-1.0, y_item-0.5), box_w-0.2, box_h, edgecolor='royalblue', facecolor='white', lw=1.5))
            ax.text(x, y_item, item, ha='center', va='center', fontsize=12)

    # 步骤之间的箭头（连接标题框中心）
    for i in range(len(steps)-1):
        x0, x1 = steps[i][1], steps[i+1][1]
        y_header = y_center + 1.5 + step_box_h/2
        ax.add_patch(FancyArrowPatch((x0+1.1, y_header), (x1-1.1, y_header), arrowstyle='->', mutation_scale=22, lw=2, color='orange'))

    # 反馈到感知的闭环箭头（精确锚定在最底部内容框边缘，避免断开）
    def col_bottom_y(yc, count):
        if count <= 0:
            return yc - 0.6
        return yc - (count-1)*1.2 - 0.5

    # 起点：反馈列最底部框的右侧边缘
    fb_x = steps[-1][1]
    fb_y = col_bottom_y(y_center, len(feedback_list))
    start = (fb_x + (box_w-0.2)/2 + 0.1, fb_y)
    # 终点：感知列最底部框的左侧边缘
    pc_x = steps[0][1]
    pc_y = col_bottom_y(y_center, len(perception_list))
    end = (pc_x - (box_w-0.2)/2 - 0.1, pc_y)


    ax.set_title("Agent结构（感知-决策-执行-反馈横向分步，可自定义内容）", fontsize=15, weight='bold')
    ax.text(0.5, 11.5, '每个步骤下内容可自定义，适合FTC复杂任务', fontsize=11, color='dimgray')

    # --- 线性流程 ---
    ax = ax2
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 3)
    ax.axis('off')
    steps = ["初始化", "移动到射击点", "预热", "射击", "移动到取球点1", "取球1", "回射击点", "射击2", "移动到取球点2", "取球2", "完成"]
    y = 1.5
    for i, step in enumerate(steps):
        ax.add_patch(Rectangle((i*1.1+0.5, y-0.4), 1.0, 0.8, edgecolor='green', facecolor='lightgreen', lw=2))
        ax.text(i*1.1+1.0, y, step, ha='center', va='center', fontsize=10)
        if i < len(steps)-1:
            ax.add_patch(FancyArrowPatch((i*1.1+1.5, y), ((i+1)*1.1+0.5, y), arrowstyle='->', mutation_scale=16, lw=2, color='green'))
    ax.set_title("传统线性流程", fontsize=15, weight='bold')
    ax.text(0.5, 2.6, '特点：顺序执行、难以自适应、扩展性差、适合简单/静态任务', fontsize=11, color='dimgray')

    plt.show()

if __name__ == "__main__":
    plot_combined()
