using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Experimental.AI;

public class ClickToMove : MonoBehaviour
{
    // Declaraci�n de una variable privada para el agente NavMesh
    private NavMeshAgent agent;

    // M�todo Start se llama al iniciar el script
    void Start()
    {
        // Obtiene y guarda una referencia al componente NavMeshAgent adjunto a este GameObject
        agent = GetComponent<NavMeshAgent>();
    }

    // M�todo Update se llama una vez por frame
    void Update()
    {
        // Comprueba si se ha hecho clic con el bot�n izquierdo del rat�n (bot�n 0)
        if (Input.GetMouseButtonDown(0))
        {
            // Crea un rayo desde la c�mara principal hacia la posici�n del rat�n en pantalla
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            // Variable para almacenar la informaci�n del objeto que golpea el rayo
            RaycastHit hit;

            // Comprueba si el rayo ha golpeado algo en el mundo
            if (Physics.Raycast(ray, out hit))
            {
                // Establece el destino del agente NavMesh al punto donde golpe� el rayo
                agent.SetDestination(hit.point);
            }
        }
    }
}

/*Fuentes de donde se consiguio ayuda para este script
 
    https://www.youtube.com/watch?v=HOAPvQONpsU
 
 */