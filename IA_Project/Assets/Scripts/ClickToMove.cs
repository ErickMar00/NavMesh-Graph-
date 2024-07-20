using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

public class ClickToMove : MonoBehaviour
{
    // Declaración de una variable privada para el agente NavMesh
    private NavMeshAgent agent;

    // Método Start se llama al iniciar el script
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
    }

    // Método Update se llama una vez por frame
    void Update()
    {
        // Comprueba si se ha hecho clic con el botón izquierdo del ratón (botón 0)
        if (Input.GetMouseButtonDown(0))
        {
            // Crea un rayo desde la cámara principal hacia la posición del ratón en pantalla
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            // Variable para almacenar la información del objeto que golpea el rayo
            RaycastHit hit;

            // Comprueba si el rayo ha golpeado algo en el mundo
            if (Physics.Raycast(ray, out hit))
            {
                // Establece el destino del agente NavMesh al punto donde golpeó el rayo
                agent.SetDestination(hit.point);
            }
        }
    }
}

/*Fuentes de donde se consiguio ayuda para este script
 
    https://www.youtube.com/watch?v=HOAPvQONpsU
 
 */