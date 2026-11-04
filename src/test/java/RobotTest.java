import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class RobotTest {

    @Test
    public void testHelloWorld() {
        System.out.println("hello world");
        // Arrange
        String message = "Hello, World!";

        // Act & Assert
        assertEquals("Hello, World!", message, "The message strings should match!");
    }
}