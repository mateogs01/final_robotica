#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

seq1 = [
            (1.0, -0.1, 0.0, 0.0),
            (1.0, -0.2, 0.0, 0.0),
            (1.0, -0.3, 0.0, 0.0),
            (1.0, -0.4, 0.0, 0.0),
            (1.0, -0.5, 0.0, 0.0),
            (1.0, -0.6, 0.0, 0.0),
            (1.0, -0.7, 0.0, 0.0),
            (1.0, -0.8, 0.0, 0.0),
            (1.0, -0.9, 0.0, 0.0),
            (1.0, -1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, -0.1, 0.0),
            (1.0, 0.0, -0.2, 0.0),
            (1.0, 0.0, -0.3, 0.0),
            (1.0, 0.0, -0.4, 0.0),
            (1.0, 0.0, -0.5, 0.0),
            (1.0, 0.0, -0.6, 0.0),
            (1.0, 0.0, -0.7, 0.0),
            (1.0, 0.0, -0.8, 0.0),
            (1.0, 0.0, -0.9, 0.0),
            (1.0, 0.0, -1.0, 0.0),
            (2.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.3),
            (1.0, 0.0, 0.0, 0.6),
            (1.0, 0.0, 0.0, 0.9),
            (1.0, 0.0, 0.0, 1.2),
            (1.0, 0.0, 0.0, 1.5),
            (1.0, 0.0, 0.0, 1.8),
            (2.0, 0.0, 0.0, 2.1),
            (2.0, 0.0, 0.0, 0.0),
            (1.0, 0.1, 0.1, 0.0),
            (1.0, 0.2, 0.2, 0.0),
            (1.0, 0.3, 0.3, 0.0),
            (1.0, 0.4, 0.4, 0.0),
            (1.0, 0.5, 0.5, 0.0),
            (1.0, 0.5, 0.5, 0.0),
            (1.0, 0.4, 0.5, 0.0),
            (1.0, 0.3, 0.5, 0.0),
            (1.0, 0.2, 0.5, 0.0),
            (1.0, 0.1, 0.5, 0.0),
            (2.0, -.5, 0.5, 0.0),
        ]

seq2 = [
            (3.0, -0.2, 0.0, 0.0),
            (3.0, 0.0, -0.2, 0.0),
            (3.0, 0.2,  0.0, 0.0),
            (3.0, 0.0,  0.2, 0.0),
            (5.0, -0.2, -0.2, 0.0),
            (5.0, 0.0, 0.0, 1.0),
            (5.0, -0.2, 0.0, 1.0),
        ]

class TimedSequenceNode(Node):
    def __init__(self):
        super().__init__('generador_velocidades')
        
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Definir la secuencia: (duración_segundos, velocidad_lineal_x, velocidad_lineal_y, velocidad_angular_z)
        self.sequence = seq2
        
        self.current_step = 0
        self.clock = self.get_clock()
        self.start_time = None  # Se inicializará cuando el clock sea válido
        self.last_published_step = -1
        self.sequence_completed = False
        self.shutting_down = False
        self.initialized = False  # Flag para indicar que ya tenemos start_time
        
        if self.get_parameter('use_sim_time').value:
            self.get_logger().info('Usando tiempo de simulación')
        else:
            self.get_logger().info('Usando tiempo de sistema')
        
        self.get_logger().info(f'Secuencia iniciada con {len(self.sequence)} pasos')
        self.create_timer(0.1, self.control_callback)


    def control_callback(self):
        if self.sequence_completed or self.shutting_down:
            return
        
        now = self.clock.now()
        current_time = now.nanoseconds / 1e9
        
        # Esperar a que el clock de simulación tenga un valor positivo
        if not self.initialized:
            if current_time > 0.0:
                self.initialized = True
                self.start_time = now
                self.get_logger().info(f'Clock de simulación listo en t={current_time:.2f}s')
                self.publish_current_step()
            return
        
        # Lógica normal de la secuencia
        if self.current_step >= len(self.sequence):
            if self.last_published_step != -2:
                self.publish_twist(0.0, 0.0, 0.0, now)
                self.last_published_step = -2
                self.sequence_completed = True
                self.get_logger().info('Secuencia completada, deteniendo robot')
                rclpy.shutdown()
            return
        
        elapsed = (now - self.start_time).nanoseconds / 1e9
        step_duration, linear_x, linear_y, angular_z = self.sequence[self.current_step]
        
        if elapsed >= step_duration:
            self.current_step += 1
            if self.current_step < len(self.sequence):
                self.start_time = now
                self.publish_current_step()
            else:
                if self.last_published_step != -2:
                    self.publish_twist(0.0, 0.0, 0.0, now)
                    self.last_published_step = -2
                    self.sequence_completed = True
                    self.get_logger().info('Secuencia completada, deteniendo robot')
                    rclpy.shutdown()


    def publish_current_step(self):
        """Publica el paso actual y actualiza last_published_step"""
        if self.current_step >= len(self.sequence):
            return
            
        now = self.clock.now()
        step_duration, linear_x, linear_y, angular_z = self.sequence[self.current_step]
        
        self.publish_twist(linear_x, linear_y, angular_z, now)
        self.last_published_step = self.current_step
        
        # Log con todas las velocidades y el tiempo actual
        self.get_logger().info(
            f'Paso {self.current_step + 1}/{len(self.sequence)}: '
            f'vx={linear_x:.2f}, vy={linear_y:.2f}, wz={angular_z:.2f} | '
            f'duración={step_duration:.1f}s | '
            f't={now.nanoseconds/1e9:.2f}s'
        )

    def publish_twist(self, linear_x, linear_y, angular_z, timestamp):
        """Publica un TwistStamped con el timestamp dado"""
        if self.shutting_down:
            return
            
        twist_msg = TwistStamped()
        twist_msg.header.stamp = timestamp.to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.angular.z = angular_z
        
        try:
            self.publisher.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando: {e}')

    def shutdown_handler(self):
        """Maneja el shutdown limpio"""
        self.shutting_down = True
        try:
            # Publicar cero una última vez
            self.publish_twist(0.0, 0.0, 0.0, self.get_clock().now())
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TimedSequenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por usuario')
        node.shutdown_handler()
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.shutdown_handler()
            rclpy.shutdown()
        node.destroy_node()
        node.get_logger().info('Nodo finalizado')

if __name__ == '__main__':
    main()
