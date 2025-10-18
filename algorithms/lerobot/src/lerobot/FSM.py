import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np

class FiniteStateMachine:
    def __init__(self):
        self.states = set()
        self.transitions = {}
        self.current_state = None
        self.initial_state = None
        self.final_states = set()
    
    def add_state(self, state, initial=False, final=False):
        """添加状态"""
        self.states.add(state)
        if initial:
            self.initial_state = state
            self.current_state = state
        if final:
            self.final_states.add(state)
    
    def add_transition(self, from_state, to_state, trigger):
        """添加状态转移"""
        if from_state not in self.transitions:
            self.transitions[from_state] = {}
        self.transitions[from_state][trigger] = to_state
    
    def process(self, input_sequence):
        """处理输入序列"""
        if self.initial_state is None:
            raise ValueError("未设置初始状态")
        
        self.current_state = self.initial_state
        path = [self.current_state]
        
        for trigger in input_sequence:
            if (self.current_state in self.transitions and 
                trigger in self.transitions[self.current_state]):
                self.current_state = self.transitions[self.current_state][trigger]
                path.append(self.current_state)
            else:
                raise ValueError(f"在状态 {self.current_state} 无法处理输入 '{trigger}'")
        
        return path
    
    def visualize(self, title="有限状态机"):
        """可视化状态机"""
        fig, ax = plt.subplots(figsize=(12, 8))
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        
        # 定义状态位置
        state_positions = {}
        num_states = len(self.states)
        
        # 为状态分配位置（圆形布局）
        if num_states > 0:
            radius = 4
            center_x, center_y = 5, 5
            
            for i, state in enumerate(self.states):
                angle = 2 * np.pi * i / num_states
                x = center_x + radius * np.cos(angle)
                y = center_y + radius * np.sin(angle)
                state_positions[state] = (x, y)
        
        # 绘制状态转移
        for from_state, transitions in self.transitions.items():
            for trigger, to_state in transitions.items():
                start_x, start_y = state_positions[from_state]
                end_x, end_y = state_positions[to_state]
                
                # 计算箭头方向
                dx = end_x - start_x
                dy = end_y - start_y
                length = np.sqrt(dx*dx + dy*dy)
                
                # 归一化
                if length > 0:
                    dx, dy = dx/length, dy/length
                
                # 调整起点和终点，避免箭头与状态圆圈重叠
                node_radius = 0.5
                start_x_adj = start_x + dx * node_radius
                start_y_adj = start_y + dy * node_radius
                end_x_adj = end_x - dx * node_radius
                end_y_adj = end_y - dy * node_radius
                
                # 绘制箭头
                ax.annotate('', xy=(end_x_adj, end_y_adj), xytext=(start_x_adj, start_y_adj),
                           arrowprops=dict(arrowstyle='->', lw=1.5, color='blue', alpha=0.7))
                
                # 添加触发器标签
                mid_x = (start_x + end_x) / 2
                mid_y = (start_y + end_y) / 2
                
                # 稍微偏移标签以避免与箭头重叠
                offset = 0.3
                label_x = mid_x - dy * offset
                label_y = mid_y + dx * offset
                
                ax.text(label_x, label_y, trigger, fontsize=10, 
                       bbox=dict(boxstyle="round,pad=0.2", facecolor="lightyellow", alpha=0.7),
                       ha='center', va='center')
        
        # 绘制状态节点
        for state, (x, y) in state_positions.items():
            # 确定状态样式
            facecolor = 'lightblue'
            edgecolor = 'darkblue'
            linewidth = 2
            
            if state == self.initial_state:
                # 初始状态有双圆圈
                inner_circle = patches.Circle((x, y), 0.4, fill=False, 
                                            edgecolor=edgecolor, linewidth=linewidth)
                ax.add_patch(inner_circle)
            
            if state in self.final_states:
                # 最终状态有双圆圈
                outer_circle = patches.Circle((x, y), 0.55, fill=False, 
                                            edgecolor=edgecolor, linewidth=linewidth)
                ax.add_patch(outer_circle)
                facecolor = 'lightgreen'  # 最终状态使用不同颜色
            
            # 绘制状态圆圈
            circle = patches.Circle((x, y), 0.5, facecolor=facecolor, 
                                  edgecolor=edgecolor, linewidth=linewidth)
            ax.add_patch(circle)
            
            # 添加状态标签
            ax.text(x, y, state, fontsize=12, fontweight='bold', 
                   ha='center', va='center')
        
        # 添加图例
        legend_elements = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                      markersize=10, label='普通状态'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightgreen', 
                      markersize=10, label='最终状态'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                      markeredgecolor='darkblue', markersize=10, markeredgewidth=2, 
                      label='初始状态'),
            plt.Line2D([0], [0], color='blue', lw=2, label='状态转移')
        ]
        
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))
        
        plt.tight_layout()
        return fig, ax

# 示例：创建一个简单的有限状态机（奇偶校验器）
def create_parity_checker():
    fsm = FiniteStateMachine()
    
    # 添加状态
    fsm.add_state("Even", initial=True)  # 偶数个1
    fsm.add_state("Odd", final=True)     # 奇数个1
    
    # 添加转移
    fsm.add_transition("Even", "Odd", "1")
    fsm.add_transition("Even", "Even", "0")
    fsm.add_transition("Odd", "Even", "1")
    fsm.add_transition("Odd", "Odd", "0")
    
    return fsm

# 示例：创建一个自动门状态机
def create_door_fsm():
    fsm = FiniteStateMachine()
    
    # 添加状态
    fsm.add_state("Closed", initial=True)
    fsm.add_state("Opening")
    fsm.add_state("Open")
    fsm.add_state("Closing")
    
    # 添加转移
    fsm.add_transition("Closed", "Opening", "button_pressed")
    fsm.add_transition("Opening", "Open", "fully_open")
    fsm.add_transition("Open", "Closing", "timeout")
    fsm.add_transition("Closing", "Closed", "fully_closed")
    fsm.add_transition("Opening", "Closing", "button_pressed")
    fsm.add_transition("Closing", "Opening", "button_pressed")
    
    return fsm

# 演示
if __name__ == "__main__":
    # 创建奇偶校验器FSM
    parity_fsm = create_parity_checker()
    
    # 测试输入序列
    test_inputs = ["1", "0", "1", "1", "0"]
    print("测试输入序列:", " -> ".join(test_inputs))
    
    try:
        path = parity_fsm.process(test_inputs)
        print("状态转移路径:", " -> ".join(path))
        print("最终状态:", path[-1])
        if path[-1] in parity_fsm.final_states:
            print("结果: 输入序列包含奇数个1")
        else:
            print("结果: 输入序列包含偶数个1")
    except ValueError as e:
        print("错误:", e)
    
    # 可视化奇偶校验器
    fig1, ax1 = parity_fsm.visualize("奇偶校验器 - 有限状态机")
    plt.show()
    
    # 创建并可视化自动门FSM
    door_fsm = create_door_fsm()
    fig2, ax2 = door_fsm.visualize("自动门控制系统 - 有限状态机")
    plt.show()